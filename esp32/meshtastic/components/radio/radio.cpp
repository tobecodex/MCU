#include "radio.h"
#include "sx1262_defs.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Import pin definitions from variant
#include "../../main/variant/heltec/variant.h"

static const char* TAG = "Radio";

RadioInterface::RadioInterface() {}
RadioInterface::~RadioInterface() {}

RadioInterface& RadioInterface::getInstance() {
    static RadioInterface instance;
    return instance;
}

void RadioInterface::init() {
    if (_initialized) return;

    ESP_LOGI(TAG, "Initializing SX1262 LoRa radio (Heltec V3)");
    ESP_LOGI(TAG, "Frequency: %u Hz (%.1f MHz)", _frequency, _frequency / 1e6f);
    ESP_LOGI(TAG, "TX Power: %d dBm", _txPower);

    // Create SPI mutex for thread-safe access
    _spi_mutex = xSemaphoreCreateMutex();
    if (!_spi_mutex) {
        ESP_LOGE(TAG, "Failed to create SPI mutex!");
        return;
    }
    ESP_LOGI(TAG, "SPI mutex created successfully");

    // Configure GPIO for RST and BUSY pins
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_RST);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // BUSY pin as input
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_BUSY);
    gpio_config(&io_conf);

    // Initialize SPI bus
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = SPI_MOSI;
    bus_config.miso_io_num = SPI_MISO;
    bus_config.sclk_io_num = SPI_CLK;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 256;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));

    // Add SX1262 device to SPI bus
    spi_device_interface_config_t dev_config = {};
    dev_config.clock_speed_hz = 10 * 1000000;  // 10 MHz (SX1262 max 16 MHz)
    dev_config.mode = 0;  // SPI mode 0
    dev_config.spics_io_num = LORA_CS;
    dev_config.queue_size = 7;
    dev_config.flags = 0;
    dev_config.pre_cb = nullptr;

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_config, &_spi));

    ESP_LOGI(TAG, "SPI bus initialized: MOSI=%d MISO=%d CLK=%d CS=%d", 
             SPI_MOSI, SPI_MISO, SPI_CLK, LORA_CS);
    ESP_LOGI(TAG, "GPIO configured: RST=%d BUSY=%d IRQ=%d", 
             LORA_RST, LORA_BUSY, LORA_IRQ);

    // Create mutex for SPI access protection
    _spi_mutex = xSemaphoreCreateMutex();
    if (_spi_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create SPI mutex");
        return;
    }

    // Initialize SX1262 chip
    reset();
    vTaskDelay(pdMS_TO_TICKS(10));
    
    setStandby();
    setPacketType(SX1262_PACKET_TYPE_LORA);
    
    // Configure for 868 MHz operation (EU/UK band)
    setRfFrequency(_frequency);
    
    // Configure PA for UK/EU compliant output (max 14 dBm)
    setPaConfig(0x04, 0x07, 0x00, 0x01);
    setTxParams(_txPower, 0x02);  // 0x02 = 40us ramp time
    
    // Set regulator to DC-DC mode for efficiency
    uint8_t regMode = SX1262_REGULATOR_DC_DC;
    spiWrite(SX1262_CMD_SET_REGULATORMODE, &regMode, 1);
    
    // Configure LoRa modulation parameters
    setModulationParams();
    
    // Configure packet parameters
    setPacketParams();
    
    // Set buffer base addresses
    setBufferBaseAddress(0x00, 0x00);
    
    // Configure DIO1 for TX and RX interrupts
    setDioIrqParams(SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE | SX1262_IRQ_TIMEOUT | SX1262_IRQ_CRC_ERROR,
                    SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE | SX1262_IRQ_TIMEOUT | SX1262_IRQ_CRC_ERROR,
                    0x0000, 0x0000);
    
    // Configure GPIO interrupt for DIO1
    gpio_config_t irq_conf = {};
    irq_conf.intr_type = GPIO_INTR_POSEDGE;
    irq_conf.mode = GPIO_MODE_INPUT;
    irq_conf.pin_bit_mask = (1ULL << LORA_IRQ);
    irq_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    irq_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&irq_conf);
    
    ESP_LOGI(TAG, "SX1262 configured: freq=%u Hz, power=%d dBm", _frequency, _txPower);

    _initialized = true;
    ESP_LOGI(TAG, "Radio initialized successfully");
}

void RadioInterface::setFrequency(uint32_t freq) {
    _frequency = freq;
    ESP_LOGI(TAG, "Frequency set to %u Hz", freq);
    // TODO: Update radio hardware frequency register
}

void RadioInterface::setTxPower(int8_t power) {
    _txPower = power;
    ESP_LOGI(TAG, "TX Power set to %d dBm", power);
    // TODO: Update radio hardware power register
}

void RadioInterface::transmit(const mesh_packet_t* packet) {
    if (!_initialized) {
        ESP_LOGW(TAG, "Radio not initialized");
        return;
    }

    ESP_LOGI(TAG, "TX: from=0x%08x to=0x%08x len=%u", packet->from, packet->to, packet->payload_len);

    // Build TX buffer: [from(4)][to(4)][payload...]
    uint8_t tx_buf[256];
    if (packet->payload_len + 8 > sizeof(tx_buf)) {
        ESP_LOGW(TAG, "Payload too large to transmit: %u bytes", packet->payload_len);
        return;
    }

    // Fill header
    tx_buf[0] = (packet->from >> 24) & 0xFF;
    tx_buf[1] = (packet->from >> 16) & 0xFF;
    tx_buf[2] = (packet->from >> 8) & 0xFF;
    tx_buf[3] = (packet->from) & 0xFF;
    tx_buf[4] = (packet->to >> 24) & 0xFF;
    tx_buf[5] = (packet->to >> 16) & 0xFF;
    tx_buf[6] = (packet->to >> 8) & 0xFF;
    tx_buf[7] = (packet->to) & 0xFF;

    if (packet->payload_len > 0 && packet->payload != nullptr) {
        memcpy(&tx_buf[8], packet->payload, packet->payload_len);
    }

    // Write data to radio buffer at offset 0
    // SX1262 WRITE_BUFFER expects [offset][data...]
    uint8_t write_buf[257];
    write_buf[0] = 0x00; // offset
    memcpy(&write_buf[1], tx_buf, packet->payload_len + 8);
    // len should include offset byte + header + payload
    spiWrite(SX1262_CMD_WRITE_BUFFER, write_buf, 1 + 8 + packet->payload_len);

    // Clear any pending IRQs before starting TX
    clearIrqStatus(0xFFFF);
    
    ESP_LOGD(TAG, "Starting TX: payload_len=%u, total_len=%u", packet->payload_len, 8 + packet->payload_len);

    // Set TX with timeout (simple ms value)
    uint32_t timeout_ms = 1000;
    uint8_t tbuf[3];
    tbuf[0] = (timeout_ms >> 16) & 0xFF;
    tbuf[1] = (timeout_ms >> 8) & 0xFF;
    tbuf[2] = timeout_ms & 0xFF;
    spiWrite(SX1262_CMD_SET_TX, tbuf, 3);

    // Wait for TX_DONE or timeout
    uint32_t wait = 0;
    uint16_t last_irq = 0;
    while (wait < 5000) { // up to ~5s
        uint16_t irq = getIrqStatus();
        if (irq != last_irq && irq != 0) {
            ESP_LOGD(TAG, "TX IRQ status: 0x%04x at %ums", irq, wait);
            last_irq = irq;
        }
        if (irq & SX1262_IRQ_TX_DONE) {
            clearIrqStatus(SX1262_IRQ_TX_DONE);
            ESP_LOGI(TAG, "TX done (irq=0x%04x after %ums)", irq, wait);
            break;
        }
        if (irq & (SX1262_IRQ_TIMEOUT | SX1262_IRQ_CRC_ERROR)) {
            clearIrqStatus(SX1262_IRQ_TIMEOUT | SX1262_IRQ_CRC_ERROR);
            ESP_LOGW(TAG, "TX reported error/timeout (irq=0x%04x after %ums)", irq, wait);
            // Continue anyway - the packet might have been sent
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        wait += 10;
    }
    
    if (wait >= 5000) {
        ESP_LOGW(TAG, "TX wait timeout - no IRQ received after 5s");
    }

    // Return to RX continuous mode after TX
    startReceiving();
}

void RadioInterface::startReceiving() {
    if (!_initialized) {
        ESP_LOGW(TAG, "Radio not initialized");
        return;
    }

    // Only log on first call or after TX - avoid spam during continuous polling
    static bool first_call = true;
    if (first_call) {
        ESP_LOGI(TAG, "Starting RX continuous mode");
        first_call = false;
    }
    
    // Clear any pending IRQs
    clearIrqStatus(SX1262_IRQ_ALL);
    
    // Set to RX continuous mode (timeout = 0xFFFFFF for continuous)
    setRx(0xFFFFFF);
}

void RadioInterface::handleReceive(const mesh_packet_t* packet) {
    ESP_LOGI(TAG, "RX: from=0x%x to=0x%x len=%u rssi=%d snr=%d",
             packet->from, packet->to, packet->payload_len, packet->rssi, packet->snr);

    // TODO: Handle received mesh packet
    // - Route packet to appropriate handler
    // - Display on OLED
    // - Log to serial output
}

bool RadioInterface::checkForPacket(mesh_packet_t* packet) {
    if (!_initialized) return false;
    
    // Check IRQ status
    uint16_t irq = getIrqStatus();
    
    // Log any non-zero IRQ for debugging
    static uint16_t last_logged_irq = 0;
    static uint32_t irq_check_count = 0;
    irq_check_count++;
    
    if (irq != 0 && irq != last_logged_irq) {
        ESP_LOGD(TAG, "RX IRQ status: 0x%04x (check #%u)", irq, irq_check_count);
        last_logged_irq = irq;
    }
    
    if (irq & SX1262_IRQ_RX_DONE) {
        // Clear IRQ
        clearIrqStatus(SX1262_IRQ_RX_DONE);
        
        // Get buffer status
        uint8_t payloadLength = 0;
        uint8_t rxStartBuffer = 0;
        getRxBufferStatus(&payloadLength, &rxStartBuffer);
        
        // Read packet
        uint8_t buffer[256];
        readBuffer(rxStartBuffer, buffer, payloadLength);
        
        // Get RSSI and SNR
        int8_t rssi, snr;
        getPacketStatus(&rssi, &snr);
        
        // Parse packet (simple format for now)
        if (payloadLength >= 8) {
            packet->from = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
            packet->to = (buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | buffer[7];
            packet->payload_len = payloadLength - 8;
            packet->rssi = rssi;
            packet->snr = snr;
            
            if (packet->payload_len > 0) {
                // Payload would be in buffer[8] onwards
                packet->payload = &buffer[8];
            }
            
            ESP_LOGI(TAG, "Packet received: from=0x%08x to=0x%08x len=%u rssi=%d snr=%d", 
                     packet->from, packet->to, packet->payload_len, rssi, snr);
            
            // Restart RX
            startReceiving();
            return true;
        }
    }
    
    if (irq & (SX1262_IRQ_TIMEOUT | SX1262_IRQ_CRC_ERROR)) {
        clearIrqStatus(SX1262_IRQ_TIMEOUT | SX1262_IRQ_CRC_ERROR);
        // Restart RX
        startReceiving();
    }
    
    return false;
}

void RadioInterface::waitOnBusy() {
    // SX1262 uses BUSY pin to indicate when it's processing a command
    uint32_t timeout = 0;
    while (gpio_get_level(LORA_BUSY) == 1 && timeout < 10000) {
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout++;
    }
    if (timeout >= 10000) {
        ESP_LOGW(TAG, "BUSY timeout");
    }
}

void RadioInterface::spiWrite(uint8_t cmd, const uint8_t* data, uint8_t len) {
    if (!_spi_mutex || xSemaphoreTake(_spi_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire SPI mutex for write");
        return;
    }
    
    waitOnBusy();
    
    spi_transaction_t t = {};
    uint8_t tx_buffer[256];
    
    tx_buffer[0] = cmd;
    if (data && len > 0) {
        memcpy(&tx_buffer[1], data, len);
    }
    
    t.length = (1 + len) * 8;  // bits
    t.tx_buffer = tx_buffer;
    t.rx_buffer = nullptr;
    
    ESP_ERROR_CHECK(spi_device_transmit(_spi, &t));
    
    xSemaphoreGive(_spi_mutex);
}

void RadioInterface::spiRead(uint8_t cmd, uint8_t* data, uint8_t len) {
    if (!_spi_mutex || xSemaphoreTake(_spi_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire SPI mutex for read");
        return;
    }
    
    waitOnBusy();
    
    spi_transaction_t t = {};
    uint8_t tx_buffer[256] = {0};
    uint8_t rx_buffer[256] = {0};
    
    // SX1262 read commands require: [CMD] [NOP] -> [STATUS] [STATUS] [DATA...]
    // We need to skip the first 2 status bytes in the response
    tx_buffer[0] = cmd;
    tx_buffer[1] = 0x00;  // NOP byte
    
    t.length = (2 + len) * 8;  // bits: cmd + nop + data length
    t.tx_buffer = tx_buffer;
    t.rx_buffer = rx_buffer;
    
    ESP_ERROR_CHECK(spi_device_transmit(_spi, &t));
    
    if (data && len > 0) {
        // Skip first 2 bytes (status bytes), copy the actual data
        memcpy(data, &rx_buffer[2], len);
    }
    
    xSemaphoreGive(_spi_mutex);
}

void RadioInterface::reset() {
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Radio reset complete");
}

void RadioInterface::setStandby() {
    uint8_t mode = SX1262_STANDBY_RC;
    spiWrite(SX1262_CMD_SET_STANDBY, &mode, 1);
    ESP_LOGI(TAG, "Set to standby mode");
}

void RadioInterface::setPacketType(uint8_t packetType) {
    spiWrite(SX1262_CMD_SET_PACKETTYPE, &packetType, 1);
    ESP_LOGI(TAG, "Packet type set to %s", packetType == SX1262_PACKET_TYPE_LORA ? "LoRa" : "GFSK");
}

void RadioInterface::setRfFrequency(uint32_t frequency) {
    // Convert frequency in Hz to SX1262 register value
    // fRF = (fXOSC * freqReg) / 2^25
    // Where fXOSC = 32 MHz
    uint32_t freqReg = (uint32_t)((double)frequency / (double)32000000.0 * (double)(1 << 25));
    
    uint8_t buf[4];
    buf[0] = (freqReg >> 24) & 0xFF;
    buf[1] = (freqReg >> 16) & 0xFF;
    buf[2] = (freqReg >> 8) & 0xFF;
    buf[3] = freqReg & 0xFF;
    
    spiWrite(SX1262_CMD_SET_RFFREQUENCY, buf, 4);
    ESP_LOGI(TAG, "RF frequency set to %u Hz (reg: 0x%08X)", frequency, freqReg);
}

void RadioInterface::setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut) {
    uint8_t buf[4];
    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    
    spiWrite(SX1262_CMD_SET_PACONFIG, buf, 4);
    ESP_LOGI(TAG, "PA config set");
}

void RadioInterface::setTxParams(int8_t power, uint8_t rampTime) {
    uint8_t buf[2];
    buf[0] = power;
    buf[1] = rampTime;
    
    spiWrite(SX1262_CMD_SET_TXPARAMS, buf, 2);
    ESP_LOGI(TAG, "TX params: power=%d dBm, ramp=%u", power, rampTime);
}

void RadioInterface::setModulationParams() {
    // LoRa modulation params: SF7, BW=125kHz, CR=4/5, LowDataRateOptimize=0
    uint8_t buf[4];
    buf[0] = 0x07;  // SF7
    buf[1] = 0x04;  // BW 125 kHz
    buf[2] = 0x01;  // CR 4/5
    buf[3] = 0x00;  // Low data rate optimize off
    
    spiWrite(SX1262_CMD_SET_MODULATIONPARAMS, buf, 4);
    ESP_LOGI(TAG, "Modulation params set: SF7, BW125, CR4/5");
}

void RadioInterface::setPacketParams() {
    // LoRa packet params: PreambleLength=8, HeaderType=explicit, PayloadLength=255, CRC=on, InvertIQ=standard
    uint8_t buf[6];
    buf[0] = 0x00;  // Preamble MSB
    buf[1] = 0x08;  // Preamble LSB (8 symbols)
    buf[2] = 0x00;  // Header type: explicit
    buf[3] = 0xFF;  // Payload length (255 max)
    buf[4] = 0x01;  // CRC on
    buf[5] = 0x00;  // InvertIQ standard
    
    spiWrite(SX1262_CMD_SET_PACKETPARAMS, buf, 6);
    ESP_LOGI(TAG, "Packet params set");
}

void RadioInterface::setBufferBaseAddress(uint8_t txBase, uint8_t rxBase) {
    uint8_t buf[2];
    buf[0] = txBase;
    buf[1] = rxBase;
    
    spiWrite(SX1262_CMD_SET_BUFFERBASEADDRESS, buf, 2);
    ESP_LOGI(TAG, "Buffer base addresses set: TX=0x%02X RX=0x%02X", txBase, rxBase);
}

void RadioInterface::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
    uint8_t buf[8];
    buf[0] = (irqMask >> 8) & 0xFF;
    buf[1] = irqMask & 0xFF;
    buf[2] = (dio1Mask >> 8) & 0xFF;
    buf[3] = dio1Mask & 0xFF;
    buf[4] = (dio2Mask >> 8) & 0xFF;
    buf[5] = dio2Mask & 0xFF;
    buf[6] = (dio3Mask >> 8) & 0xFF;
    buf[7] = dio3Mask & 0xFF;
    
    spiWrite(SX1262_CMD_CFG_DIOIRQ, buf, 8);
    ESP_LOGI(TAG, "DIO IRQ params configured");
}

void RadioInterface::setRx(uint32_t timeout) {
    uint8_t buf[3];
    buf[0] = (timeout >> 16) & 0xFF;
    buf[1] = (timeout >> 8) & 0xFF;
    buf[2] = timeout & 0xFF;
    
    spiWrite(SX1262_CMD_SET_RX, buf, 3);
}

void RadioInterface::getRxBufferStatus(uint8_t* payloadLength, uint8_t* rxStartBuffer) {
    uint8_t status[2];
    spiRead(SX1262_CMD_GET_RXBUFFERSTATUS, status, 2);
    *payloadLength = status[0];
    *rxStartBuffer = status[1];
}

void RadioInterface::readBuffer(uint8_t offset, uint8_t* data, uint8_t len) {
    waitOnBusy();
    
    spi_transaction_t t = {};
    uint8_t tx_buffer[258] = {0};
    uint8_t rx_buffer[258] = {0};
    
    tx_buffer[0] = SX1262_CMD_READ_BUFFER;
    tx_buffer[1] = offset;
    tx_buffer[2] = 0x00;  // NOP
    
    t.length = (3 + len) * 8;
    t.tx_buffer = tx_buffer;
    t.rx_buffer = rx_buffer;
    
    ESP_ERROR_CHECK(spi_device_transmit(_spi, &t));
    
    if (data && len > 0) {
        memcpy(data, &rx_buffer[3], len);
    }
}

void RadioInterface::getPacketStatus(int8_t* rssi, int8_t* snr) {
    uint8_t status[3];
    spiRead(SX1262_CMD_GET_PACKETSTATUS, status, 3);
    
    // For LoRa: status[0] = RssiPkt, status[1] = SnrPkt, status[2] = SignalRssiPkt
    *rssi = -status[0] / 2;  // Convert to dBm
    *snr = (int8_t)status[1] / 4;  // Convert to dB
}

uint16_t RadioInterface::getIrqStatus() {
    // SX1262 GET_IRQSTATUS: send [CMD][NOP], receive [STATUS][IRQ_MSB][IRQ_LSB]
    // According to datasheet we need NOP byte
    waitOnBusy();
    
    spi_transaction_t t = {};
    uint8_t tx_buffer[4] = {SX1262_CMD_GET_IRQSTATUS, 0x00, 0x00, 0x00};
    uint8_t rx_buffer[4] = {0};
    
    t.length = 4 * 8;  // 4 bytes to be safe
    t.tx_buffer = tx_buffer;
    t.rx_buffer = rx_buffer;
    
    if (!_spi_mutex || xSemaphoreTake(_spi_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire SPI mutex for IRQ status");
        return 0;
    }
    
    ESP_ERROR_CHECK(spi_device_transmit(_spi, &t));
    
    xSemaphoreGive(_spi_mutex);
    
    // Debug: log all bytes received
    static bool first_time = true;
    if (first_time) {
        ESP_LOGI(TAG, "IRQ raw RX bytes: [0]=0x%02x [1]=0x%02x [2]=0x%02x [3]=0x%02x", 
                 rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
        first_time = false;
    }
    
    // rx_buffer[0] = status, rx_buffer[1] = status, rx_buffer[2] = IRQ LSB?, rx_buffer[3] = IRQ MSB?
    // Try LSB first based on actual data
    uint16_t irq = (rx_buffer[3] << 8) | rx_buffer[2];
    
    // Log IRQ bits for debugging (only if non-zero)
    static uint16_t last_nonzero_irq = 0;
    if (irq != 0 && irq != last_nonzero_irq) {
        ESP_LOGI(TAG, "IRQ status: 0x%04x (TX_DONE=%d RX_DONE=%d TIMEOUT=%d)", 
                 irq,
                 (irq & SX1262_IRQ_TX_DONE) ? 1 : 0,
                 (irq & SX1262_IRQ_RX_DONE) ? 1 : 0,
                 (irq & SX1262_IRQ_TIMEOUT) ? 1 : 0);
        last_nonzero_irq = irq;
    }
    
    return irq;
}

void RadioInterface::clearIrqStatus(uint16_t irq) {
    uint8_t buf[2];
    buf[0] = (irq >> 8) & 0xFF;
    buf[1] = irq & 0xFF;
    
    spiWrite(SX1262_CMD_CLR_IRQSTATUS, buf, 2);
}

RadioInterface& radio = RadioInterface::getInstance();
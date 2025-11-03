#include "radio.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Import pin definitions from variant
#include "../../main/variant/heltec/variant.h"

static const char* TAG = "Radio";

// SX1262 command opcodes
#define SX1262_CMD_SET_SLEEP              0x84
#define SX1262_CMD_SET_STANDBY            0x80
#define SX1262_CMD_SET_FS                 0xC1
#define SX1262_CMD_SET_TX                 0x83
#define SX1262_CMD_SET_RX                 0x82
#define SX1262_CMD_SET_RXDUTYCYCLE        0x94
#define SX1262_CMD_SET_CAD                0xC5
#define SX1262_CMD_SET_TXCONTINUOUSWAVE   0xD1
#define SX1262_CMD_SET_TXCONTINUOUSPREAMBLE 0xD2
#define SX1262_CMD_SET_PACKETTYPE         0x8A
#define SX1262_CMD_GET_PACKETTYPE         0x11
#define SX1262_CMD_SET_RFFREQUENCY        0x86
#define SX1262_CMD_SET_TXPARAMS           0x8E
#define SX1262_CMD_SET_PACONFIG           0x95
#define SX1262_CMD_SET_CADPARAMS          0x88
#define SX1262_CMD_SET_BUFFERBASEADDRESS  0x8F
#define SX1262_CMD_SET_MODULATIONPARAMS   0x8B
#define SX1262_CMD_SET_PACKETPARAMS       0x8C
#define SX1262_CMD_GET_RXBUFFERSTATUS     0x13
#define SX1262_CMD_GET_PACKETSTATUS       0x14
#define SX1262_CMD_GET_RSSIINST           0x15
#define SX1262_CMD_GET_STATS              0x10
#define SX1262_CMD_RESET_STATS            0x00
#define SX1262_CMD_CFG_DIOIRQ             0x08
#define SX1262_CMD_GET_IRQSTATUS          0x12
#define SX1262_CMD_CLR_IRQSTATUS          0x02
#define SX1262_CMD_CALIBRATE              0x89
#define SX1262_CMD_CALIBRATEIMAGE         0x98
#define SX1262_CMD_SET_REGULATORMODE      0x96
#define SX1262_CMD_GET_ERROR              0x17
#define SX1262_CMD_CLR_ERROR              0x07
#define SX1262_CMD_SET_TCXOMODE           0x97
#define SX1262_CMD_SET_TXFALLBACKMODE     0x93
#define SX1262_CMD_SET_RFSWITCHMODE       0x9D
#define SX1262_CMD_SET_STOPRXTIMERONPREAMBLE 0x9F
#define SX1262_CMD_SET_LORASYMBTIMEOUT    0xA0
#define SX1262_CMD_GET_STATUS             0xC0
#define SX1262_CMD_WRITE_REGISTER         0x0D
#define SX1262_CMD_READ_REGISTER          0x1D
#define SX1262_CMD_WRITE_BUFFER           0x0E
#define SX1262_CMD_READ_BUFFER            0x1E

// Packet types
#define SX1262_PACKET_TYPE_GFSK           0x00
#define SX1262_PACKET_TYPE_LORA           0x01

// Standby modes
#define SX1262_STANDBY_RC                 0x00
#define SX1262_STANDBY_XOSC               0x01

// Regulator modes
#define SX1262_REGULATOR_LDO              0x00
#define SX1262_REGULATOR_DC_DC            0x01

// IRQ masks
#define SX1262_IRQ_TX_DONE                0x0001
#define SX1262_IRQ_RX_DONE                0x0002
#define SX1262_IRQ_PREAMBLE_DETECTED      0x0004
#define SX1262_IRQ_SYNC_WORD_VALID        0x0008
#define SX1262_IRQ_HEADER_VALID           0x0010
#define SX1262_IRQ_HEADER_ERROR           0x0020
#define SX1262_IRQ_CRC_ERROR              0x0040
#define SX1262_IRQ_CAD_DONE               0x0080
#define SX1262_IRQ_CAD_DETECTED           0x0100
#define SX1262_IRQ_TIMEOUT                0x0200
#define SX1262_IRQ_ALL                    0x03FF

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
    
    // Configure DIO1 for RX done interrupt
    setDioIrqParams(SX1262_IRQ_RX_DONE | SX1262_IRQ_TIMEOUT | SX1262_IRQ_CRC_ERROR,
                    SX1262_IRQ_RX_DONE | SX1262_IRQ_TIMEOUT | SX1262_IRQ_CRC_ERROR,
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

    ESP_LOGI(TAG, "TX: from=0x%x to=0x%x len=%u rssi=%d snr=%d",
             packet->from, packet->to, packet->payload_len, packet->rssi, packet->snr);

    // TODO: Implement LoRa packet transmission
}

void RadioInterface::startReceiving() {
    if (!_initialized) {
        ESP_LOGW(TAG, "Radio not initialized");
        return;
    }

    ESP_LOGI(TAG, "Starting RX continuous mode");
    
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
}

void RadioInterface::spiRead(uint8_t cmd, uint8_t* data, uint8_t len) {
    waitOnBusy();
    
    spi_transaction_t t = {};
    uint8_t tx_buffer[256] = {0};
    uint8_t rx_buffer[256] = {0};
    
    tx_buffer[0] = cmd;
    
    t.length = (1 + len) * 8;  // bits
    t.tx_buffer = tx_buffer;
    t.rx_buffer = rx_buffer;
    
    ESP_ERROR_CHECK(spi_device_transmit(_spi, &t));
    
    if (data && len > 0) {
        memcpy(data, &rx_buffer[1], len);
    }
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
    uint8_t status[2];
    spiRead(SX1262_CMD_GET_IRQSTATUS, status, 2);
    return (status[0] << 8) | status[1];
}

void RadioInterface::clearIrqStatus(uint16_t irq) {
    uint8_t buf[2];
    buf[0] = (irq >> 8) & 0xFF;
    buf[1] = irq & 0xFF;
    
    spiWrite(SX1262_CMD_CLR_IRQSTATUS, buf, 2);
}

RadioInterface& radio = RadioInterface::getInstance();
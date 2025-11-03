#include "radio.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

    // TODO: Initialize SPI bus and LoRa radio hardware
    // - Configure SPI for radio communication
    // - Initialize SX1262 registers (different from SX127x!)
    // - Set frequency and power
    // - Configure interrupt for packet reception
    // - Handle BUSY pin (SX1262 specific)

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

    ESP_LOGI(TAG, "Starting RX mode");
    // TODO: Implement continuous receive mode with interrupt handling
}

void RadioInterface::handleReceive(const mesh_packet_t* packet) {
    ESP_LOGI(TAG, "RX: from=0x%x to=0x%x len=%u rssi=%d snr=%d",
             packet->from, packet->to, packet->payload_len, packet->rssi, packet->snr);

    // TODO: Handle received mesh packet
    // - Route packet to appropriate handler
    // - Display on OLED
    // - Log to serial output
}

RadioInterface& radio = RadioInterface::getInstance();
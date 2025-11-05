#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "version.h"
#include "driver/gpio.h"

// Components
#include "../components/display/display.h"
#include "../components/radio/radio.h"

static const char* TAG = "Main";

// Heltec V3 hardware
#include "variant/heltec/variant.h"

void blink_task(void* param) {
    gpio_config_t gpio_conf = {};
    gpio_conf.pin_bit_mask = (1ULL << LED_PIN);
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio_conf);

    while (1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(20));  // Very brief flash (20ms)
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(5000));  // Long pause (5 seconds)
    }
}

void radio_receive_task(void* param) {
    // Task to handle incoming mesh messages
    vTaskDelay(pdMS_TO_TICKS(2000));  // Allow radio to initialize

    ESP_LOGI(TAG, "Radio RX task started");
    ESP_LOGI(TAG, "Frequency: %u Hz, Power: %d dBm", radio.getFrequency(), radio.getTxPower());
    
    // Start receiving
    radio.startReceiving();

    mesh_packet_t packet = {};
    uint32_t packet_count = 0;
    uint32_t check_count = 0;
    
    while (1) {
        check_count++;
        
        // Log every 200 checks (20 seconds) to show we're alive
        if (check_count % 200 == 0) {
            ESP_LOGI(TAG, "Radio listening... checked %u times, received %u packets", 
                     check_count, packet_count);
        }
        
        // Check for received packets
        if (radio.checkForPacket(&packet)) {
            packet_count++;
            
            ESP_LOGI(TAG, "Packet #%u received from 0x%08x to 0x%08x, len=%u, RSSI=%d, SNR=%d",
                     packet_count, packet.from, packet.to, packet.payload_len, packet.rssi, packet.snr);
            
            // Update display with packet info
            char msg[32];
            snprintf(msg, sizeof(msg), "RX-%" PRIu32 " RSSI:%ddBm", packet_count, packet.rssi);
            display.showMessage(msg);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
    }
}

void send_task(void* param) {
    // Periodically send a simple Meshtastic "Hello" message
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait for radio/display init

    uint32_t send_count = 0;
    uint8_t mac_local[6] = {0};
    esp_read_mac(mac_local, ESP_MAC_WIFI_STA);
    
    // Use last 4 bytes of MAC as node ID for uniqueness
    uint32_t node_id = ((uint32_t)mac_local[2] << 24) | 
                       ((uint32_t)mac_local[3] << 16) | 
                       ((uint32_t)mac_local[4] << 8) | 
                       ((uint32_t)mac_local[5]);
    
    ESP_LOGI(TAG, "Node ID: 0x%08x (from MAC)", node_id);
    
    while (1) {
        char payload[64];
        snprintf(payload, sizeof(payload), "Hello %02x%02x #%" PRIu32,
                 mac_local[4], mac_local[5], send_count++);

        mesh_packet_t packet = {};
        packet.from = node_id;           // Use unique node ID
        packet.to = 0xFFFFFFFF;          // broadcast
        packet.want_ack = 0;
        packet.hop_limit = 3;
        packet.payload = (uint8_t*)payload;
        packet.payload_len = (uint16_t)strlen(payload);

        ESP_LOGI(TAG, "Sending hello packet #%" PRIu32 ", len=%u", send_count, packet.payload_len);
        radio.transmit(&packet);

        // Send every 15 seconds
        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Meshtastic Node - Heltec V3");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Firmware: %s (Built %s %s)", FW_VERSION, FW_BUILD_DATE, FW_BUILD_TIME);

    // Initialize NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition full or new version, erasing and reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Board: %s", BOARD_NAME);
    ESP_LOGI(TAG, "GPIO Config:");
    ESP_LOGI(TAG, "  I2C SDA=%d SCL=%d", I2C_SDA, I2C_SCL);
    ESP_LOGI(TAG, "  LoRa CS=%d IRQ=%d RST=%d", LORA_CS, LORA_IRQ, LORA_RST);
    ESP_LOGI(TAG, "  LED=%d", LED_PIN);

    // Initialize display first to show boot status
    ESP_LOGI(TAG, "Initializing display...");
    display.init();
    display.showBootScreen();
    display.showStatus("Boot: Display OK");

    // Initialize radio
    ESP_LOGI(TAG, "Initializing radio...");
    radio.init();
    radio.startReceiving();
    display.showStatus("Boot: Radio OK");

    // Get system info and display
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    uint32_t heap_free = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Free heap: %u bytes", heap_free);

    // Ensure LED is off (no blinking/feedback)
    {
        gpio_config_t gpio_conf = {};
        gpio_conf.pin_bit_mask = (1ULL << LED_PIN);
        gpio_conf.mode = GPIO_MODE_OUTPUT;
        gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&gpio_conf);
        gpio_set_level(LED_PIN, 0);
    }

    // Start radio receive task to log incoming messages
    xTaskCreate(radio_receive_task, "radio_rx", 4096, NULL, 5, NULL);

    // Start periodic send task (Hello world)
    xTaskCreate(send_task, "sender", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Meshtastic node ready - waiting for mesh messages...");
    
    // Keep main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
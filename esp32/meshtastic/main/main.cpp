#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Components
#include "../components/display/display.h"
#include "../components/radio/radio.h"

static const char* TAG = "Main";

// Heltec V3 hardware
#include "variant/heltec/variant.h"

void blink_task(void* param) {
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&gpio_conf);

    while (1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(900));
    }
}

void radio_receive_task(void* param) {
    // Task to handle incoming mesh messages
    // This will continuously listen for LoRa packets and log them

    vTaskDelay(pdMS_TO_TICKS(2000));  // Allow radio to initialize

    ESP_LOGI(TAG, "Radio RX task started");
    display.showStatus("RX: Listening...");

    while (1) {
        // TODO: Wait for incoming packets from radio
        // For now, just a placeholder that shows it's listening
        vTaskDelay(pdMS_TO_TICKS(5000));
        display.showStatus("RX: Listening...");
    }
}

void mesh_task(void* param) {
    // Main mesh networking task
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Mesh task started");
    display.showStatus("Mesh: Running");

    uint32_t last_send = esp_log_timestamp();

    while (1) {
        uint32_t now = esp_log_timestamp();

        // Example: Periodically show node status
        if (now - last_send > 30000) {
            ESP_LOGI(TAG, "Node is alive - ready to relay messages");
            display.showStatus("Mesh: Active");
            last_send = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Meshtastic Node - Heltec V3");
    ESP_LOGI(TAG, "========================================");

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

    // Start LED blink task (indicates system running)
    xTaskCreate(blink_task, "blink", 512, NULL, 1, NULL);

    // Start radio receive task to log incoming messages
    xTaskCreate(radio_receive_task, "radio_rx", 2048, NULL, 5, NULL);

    // Start main mesh task
    xTaskCreate(mesh_task, "mesh", 2048, NULL, 5, NULL);

    display.showStatus("Boot: Complete");
    ESP_LOGI(TAG, "Meshtastic node ready - waiting for mesh messages...");
}
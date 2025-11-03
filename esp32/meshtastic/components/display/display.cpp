#include "display.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "Display";

// Simple SSD1306 OLED controller - minimal implementation
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_FREQ_HZ 400000
#define OLED_I2C_ADDR 0x3c

#include "../../main/variant/heltec/variant.h"

static uint8_t framebuffer[1024] = {0};  // 128x64 / 8

Display::Display() {}
Display::~Display() {}

Display& Display::getInstance() {
    static Display instance;
    return instance;
}

void Display::init() {
    if (_initialized) return;

    ESP_LOGI(TAG, "Initializing I2C for OLED display");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = I2C_FREQ_HZ},
        .clk_flags = 0,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    _initialized = true;
    showBootScreen();
}

void Display::showBootScreen() {
    clear();
    ESP_LOGI(TAG, "Display: Boot screen");
}

void Display::showStatus(const char* status) {
    if (!_initialized) return;
    ESP_LOGI(TAG, "Display: %s", status);
}

void Display::showMessage(const char* message) {
    if (!_initialized) return;
    ESP_LOGI(TAG, "Display: Message: %s", message);
}

void Display::showSignal(int rssi) {
    if (!_initialized) return;
    ESP_LOGI(TAG, "Display: Signal RSSI: %d", rssi);
}

void Display::showNodeInfo(uint32_t nodeId, const char* longName) {
    if (!_initialized) return;
    ESP_LOGI(TAG, "Display: Node 0x%x - %s", nodeId, longName);
}

void Display::clear() {
    if (!_initialized) return;
    memset(framebuffer, 0, sizeof(framebuffer));
}

Display& display = Display::getInstance();
#include "display.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

static const char* TAG = "Display";

// Simple SSD1306 OLED controller - minimal implementation
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_FREQ_HZ 400000
#define OLED_I2C_ADDR 0x3c

// SSD1306 commands
#define SSD1306_CMD_DISPLAY_OFF       0xAE
#define SSD1306_CMD_DISPLAY_ON        0xAF
#define SSD1306_CMD_SET_CONTRAST      0x81
#define SSD1306_CMD_SET_PRECHARGE     0xD9
#define SSD1306_CMD_SET_VCOM_DESELECT 0xDB
#define SSD1306_CMD_SET_DISPLAY_CLOCK 0xD5
#define SSD1306_CMD_SET_MULTIPLEX     0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_CMD_SET_START_LINE    0x40
#define SSD1306_CMD_CHARGE_PUMP       0x8D
#define SSD1306_CMD_MEMORY_MODE       0x20
#define SSD1306_CMD_SEG_REMAP         0xA1
#define SSD1306_CMD_COM_SCAN_DEC      0xC8
#define SSD1306_CMD_SET_COM_PINS      0xDA
#define SSD1306_CMD_SET_COLUMN_ADDR   0x21
#define SSD1306_CMD_SET_PAGE_ADDR     0x22

#include "../../main/variant/heltec/variant.h"
#include "driver/gpio.h"

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

    // Enable Vext power for OLED (Heltec V3 requirement)
    // Vext is active LOW - set to 0 to enable power
    gpio_config_t vext_conf = {};
    vext_conf.pin_bit_mask = (1ULL << VEXT_CTRL);
    vext_conf.mode = GPIO_MODE_OUTPUT;
    vext_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    vext_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    vext_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&vext_conf);
    
    gpio_set_level(VEXT_CTRL, 0);  // LOW = power ON
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for power to stabilize
    ESP_LOGI(TAG, "Vext power enabled for OLED");

    // Reset the OLED display (hardware reset)
    gpio_config_t rst_conf = {};
    rst_conf.pin_bit_mask = (1ULL << OLED_RST);
    rst_conf.mode = GPIO_MODE_OUTPUT;
    rst_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    rst_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    rst_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&rst_conf);
    
    // Pulse reset: LOW for 10ms, then HIGH
    gpio_set_level(OLED_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(OLED_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "OLED hardware reset complete");

    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {};
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_MASTER_NUM;
    bus_config.scl_io_num = I2C_SCL;
    bus_config.sda_io_num = I2C_SDA;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &_bus_handle));

    // Add OLED device to the bus
    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = OLED_I2C_ADDR;
    dev_config.scl_speed_hz = I2C_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus_handle, &dev_config, &_dev_handle));

    // Test I2C communication
    uint8_t test_data[1] = {0x00};
    esp_err_t ret = i2c_master_transmit(_dev_handle, test_data, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C probe failed: %s (0x%x)", esp_err_to_name(ret), ret);
        ESP_LOGE(TAG, "Display not responding at address 0x%02x!", OLED_I2C_ADDR);
        return;
    }
    ESP_LOGI(TAG, "I2C device found at 0x%02x", OLED_I2C_ADDR);

    // Initialize SSD1306 OLED
    initSSD1306();

    _initialized = true;
    showBootScreen();
}

void Display::writeCommand(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd};  // 0x00 = command mode
    esp_err_t err = i2c_master_transmit(_dev_handle, data, 2, 1000);  // 1 second timeout
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C command failed: %s", esp_err_to_name(err));
    }
}

void Display::writeData(const uint8_t* data, size_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = 0x40;  // 0x40 = data mode
    memcpy(&buffer[1], data, len);
    esp_err_t err = i2c_master_transmit(_dev_handle, buffer, len + 1, 1000);  // 1 second timeout
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C data write failed: %s", esp_err_to_name(err));
    }
}

void Display::initSSD1306() {
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for OLED to power up

    ESP_LOGI(TAG, "Sending SSD1306 init sequence...");
    
    writeCommand(SSD1306_CMD_DISPLAY_OFF);
    writeCommand(SSD1306_CMD_SET_DISPLAY_CLOCK);
    writeCommand(0x80);
    writeCommand(SSD1306_CMD_SET_MULTIPLEX);
    writeCommand(0x3F);  // 64 rows
    writeCommand(SSD1306_CMD_SET_DISPLAY_OFFSET);
    writeCommand(0x00);
    writeCommand(SSD1306_CMD_SET_START_LINE | 0x00);
    writeCommand(SSD1306_CMD_CHARGE_PUMP);
    writeCommand(0x14);  // Enable charge pump
    writeCommand(SSD1306_CMD_MEMORY_MODE);
    writeCommand(0x00);  // Horizontal addressing mode
    writeCommand(SSD1306_CMD_SEG_REMAP | 0x01);  // Column 127 is SEG0
    writeCommand(SSD1306_CMD_COM_SCAN_DEC);      // Scan from COM[N-1] to COM0
    writeCommand(SSD1306_CMD_SET_COM_PINS);
    writeCommand(0x12);
    writeCommand(SSD1306_CMD_SET_CONTRAST);
    writeCommand(0xFF);  // Maximum contrast
    writeCommand(SSD1306_CMD_SET_PRECHARGE);
    writeCommand(0xF1);
    writeCommand(SSD1306_CMD_SET_VCOM_DESELECT);
    writeCommand(0x40);
    writeCommand(0xA4);  // Display from RAM (not all ON)
    writeCommand(0xA6);  // Normal display (not inverted)
    
    // Clear display RAM before turning on
    clear();
    update();
    
    ESP_LOGI(TAG, "Turning display ON...");
    writeCommand(SSD1306_CMD_DISPLAY_ON);
    
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "SSD1306 initialized");
}

void Display::update() {
    // Set column address (0-127)
    writeCommand(SSD1306_CMD_SET_COLUMN_ADDR);
    writeCommand(0);
    writeCommand(127);

    // Set page address (0-7 for 64 rows)
    writeCommand(SSD1306_CMD_SET_PAGE_ADDR);
    writeCommand(0);
    writeCommand(7);

    // Send framebuffer in chunks
    for (int i = 0; i < 1024; i += 32) {
        writeData(&framebuffer[i], 32);
    }
}

// Simple 5x7 font (only uppercase A-Z, 0-9, and a few symbols)
static const uint8_t font5x7[][5] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x00, 0x00, 0x00, 0x00}, // space
    {0x00, 0x00, 0x2F, 0x00, 0x00}, // !
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
};

void Display::drawChar(uint8_t x, uint8_t y, char c) {
    if (x >= 128 || y >= 8) return;

    int idx = -1;
    if (c >= 'A' && c <= 'Z') idx = c - 'A';
    else if (c >= 'a' && c <= 'z') idx = c - 'a';  // Treat lowercase as uppercase
    else if (c >= '0' && c <= '9') idx = 26 + (c - '0');
    else if (c == ' ') idx = 36;
    else if (c == '!') idx = 37;
    else if (c == ':') idx = 38;
    else if (c == '-') idx = 39;

    if (idx < 0 || idx >= 40) return;

    // Font is 5 pixels wide, each byte represents a vertical column
    // Each bit in the byte represents a pixel (bit 0 = top, bit 6 = bottom for 7-pixel height)
    for (int col = 0; col < 5; col++) {
        if (x + col >= 128) break;
        
        uint8_t column_data = font5x7[idx][col];
        
        // The font data bits represent vertical pixels
        // We need to place them in the framebuffer which uses pages (8 pixels per page)
        // y is the page number (0-7), and we write the column data directly
        framebuffer[y * 128 + x + col] = column_data;
    }
}

void Display::drawText(uint8_t x, uint8_t y, const char* text) {
    uint8_t xpos = x;
    for (int i = 0; text[i] != '\0'; i++) {
        drawChar(xpos, y, text[i]);
        xpos += 6;  // 5 pixels + 1 space
        if (xpos >= 128) break;
    }
}

void Display::showBootScreen() {
    clear();
    
    snprintf(_line1, sizeof(_line1), "MESHTASTIC");
    snprintf(_line2, sizeof(_line2), "ESP32-S3");
    snprintf(_line3, sizeof(_line3), "LoRa 868MHz UK");
    snprintf(_line4, sizeof(_line4), "Booting...");
    
    drawText(20, 1, _line1);
    drawText(25, 3, _line2);
    drawText(15, 5, _line3);
    drawText(10, 7, _line4);
    update();
    
    ESP_LOGI(TAG, "Boot screen displayed");
}

void Display::showStatus(const char* status) {
    if (!_initialized) return;
    
    snprintf(_line4, sizeof(_line4), "%s", status);
    clear();
    drawText(20, 1, "MESHTASTIC");
    drawText(0, 3, _line2);
    drawText(0, 5, _line3);
    drawText(0, 7, _line4);
    update();
    
    ESP_LOGI(TAG, "Status: %s", status);
}

void Display::showMessage(const char* message) {
    if (!_initialized) return;
    
    // Show packet received notification prominently
    clear();
    drawText(10, 0, "- PACKET RX -");
    drawText(0, 2, message);
    drawText(0, 4, "868 MHz EU/UK");
    drawText(0, 6, "Mesh Active");
    update();
    
    ESP_LOGI(TAG, "Message: %s", message);
}

void Display::showSignal(int rssi) {
    if (!_initialized) return;
    
    snprintf(_line2, sizeof(_line2), "RSSI: %ddBm", rssi);
    snprintf(_line3, sizeof(_line3), "Packets: RX");
    clear();
    drawText(20, 1, "MESHTASTIC");
    drawText(0, 3, _line2);
    drawText(0, 5, _line3);
    drawText(0, 7, "Listening...");
    update();
    
    ESP_LOGI(TAG, "Signal RSSI: %d dBm", rssi);
}

void Display::showNodeInfo(uint32_t nodeId, const char* longName) {
    if (!_initialized) return;
    
    snprintf(_line2, sizeof(_line2), "Node: %08" PRIX32, nodeId);
    snprintf(_line3, sizeof(_line3), "%s", longName);
    clear();
    drawText(20, 1, "MESHTASTIC");
    drawText(0, 3, _line2);
    drawText(0, 5, _line3);
    drawText(0, 7, _line4);
    update();
    
    ESP_LOGI(TAG, "Node 0x%08" PRIX32 " - %s", nodeId, longName);
}

void Display::clear() {
    if (!_initialized) return;
    memset(framebuffer, 0, sizeof(framebuffer));
}

Display& display = Display::getInstance();
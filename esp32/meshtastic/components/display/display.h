#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"

class Display {
public:
    static Display& getInstance();

    void init();
    void showBootScreen();
    void showStatus(const char* status);
    void showMessage(const char* message);
    void showSignal(int rssi);
    void showNodeInfo(uint32_t nodeId, const char* longName);
    void clear();

private:
    Display();
    ~Display();

    void writeCommand(uint8_t cmd);
    void writeData(const uint8_t* data, size_t len);
    void initSSD1306();
    void update();
    void drawText(uint8_t x, uint8_t y, const char* text);
    void drawChar(uint8_t x, uint8_t y, char c);

    bool _initialized = false;
    i2c_master_bus_handle_t _bus_handle = nullptr;
    i2c_master_dev_handle_t _dev_handle = nullptr;
    char _line1[32] = {0};
    char _line2[32] = {0};
    char _line3[32] = {0};
    char _line4[32] = {0};
};

extern Display& display;
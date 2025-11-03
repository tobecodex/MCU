#pragma once

#include <stdint.h>

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

    bool _initialized = false;
};

extern Display& display;
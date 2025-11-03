#pragma once

#include <stdint.h>

typedef struct {
    uint32_t from;
    uint32_t to;
    uint8_t want_ack;
    uint8_t hop_limit;
    uint8_t* payload;
    uint16_t payload_len;
    int8_t rssi;
    int8_t snr;
} mesh_packet_t;

class RadioInterface {
public:
    static RadioInterface& getInstance();

    void init();
    void setFrequency(uint32_t freq);
    void setTxPower(int8_t power);
    void transmit(const mesh_packet_t* packet);
    void startReceiving();
    void handleReceive(const mesh_packet_t* packet);

    uint32_t getFrequency() const { return _frequency; }
    int8_t getTxPower() const { return _txPower; }

private:
    RadioInterface();
    ~RadioInterface();

    uint32_t _frequency = 915000000;  // 915 MHz default
    int8_t _txPower = 17;
    bool _initialized = false;
};

extern RadioInterface& radio;
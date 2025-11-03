#pragma once

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

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
    bool checkForPacket(mesh_packet_t* packet);  // Returns true if packet received

    uint32_t getFrequency() const { return _frequency; }
    int8_t getTxPower() const { return _txPower; }

private:
    RadioInterface();
    ~RadioInterface();

    void waitOnBusy();
    void reset();
    void setStandby();
    void setPacketType(uint8_t packetType);
    void setRfFrequency(uint32_t frequency);
    void setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);
    void setTxParams(int8_t power, uint8_t rampTime);
    void setModulationParams();
    void setPacketParams();
    void setBufferBaseAddress(uint8_t txBase, uint8_t rxBase);
    void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    void setRx(uint32_t timeout);
    void getRxBufferStatus(uint8_t* payloadLength, uint8_t* rxStartBuffer);
    void readBuffer(uint8_t offset, uint8_t* data, uint8_t len);
    void getPacketStatus(int8_t* rssi, int8_t* snr);
    uint16_t getIrqStatus();
    void clearIrqStatus(uint16_t irq);
    void spiWrite(uint8_t cmd, const uint8_t* data, uint8_t len);
    void spiRead(uint8_t cmd, uint8_t* data, uint8_t len);

    uint32_t _frequency = 868000000;  // 868 MHz - EU/UK band (was 915 MHz US)
    int8_t _txPower = 14;  // 14 dBm - UK/EU compliant (was 17 dBm)
    bool _initialized = false;
    
    spi_device_handle_t _spi = nullptr;
};

extern RadioInterface& radio;
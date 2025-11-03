# Quick Start Guide

## Your Setup

**Detected Hardware:**
- Board: Likely **Heltec WiFi LoRa 32** or **T-Beam** (blue OLED display confirms this)
- Serial Interface: **Silicon Labs CP210x** on `/dev/ttyUSB0`
- Baud Rate: **921600**

## First Build

```bash
cd /home/tobe/src/esp32/meshtastic

# For Heltec board (recommended if unsure)
make heltec
make build

# Or for T-Beam board
make tbeam
make build
```

## Flash to Device

```bash
make flash PORT=/dev/ttyUSB0
```

## Monitor Messages

```bash
# View serial output in real-time
make monitor

# Or with custom baud rate (if needed)
idf.py monitor -p /dev/ttyUSB0 -b 921600
```

## Expected Output

You should see:
1. ESP-IDF boot messages
2. Board initialization ("Meshtastic Node - Heltec")
3. GPIO configuration details
4. "Meshtastic node ready - waiting for mesh messages..."
5. Display shows "Boot: Complete"
6. LED blinks continuously (hardware verification)

## Viewing Incoming Messages

The serial output will show all received mesh packets:

```
I (5432) Radio: RX: from=0x12345678 to=0xffffffff len=32 rssi=-105 snr=5
I (5433) Display: Display: Message from 0x12345678
```

The blue OLED display also shows:
- "RX: Listening..." - Ready to receive
- "Mesh: Active" - Node is relaying
- Recent message info

## Troubleshooting

### Can't flash?
```bash
# List available ports
ls -l /dev/ttyUSB*

# Try specific port
make flash PORT=/dev/ttyUSB0

# Check permissions (if denied)
sudo usermod -aG dialout $USER
# Then log out and back in
```

### Nothing appears on display?
- Verify board variant is correct (Heltec vs T-Beam)
- Check I2C pins in `main/variant/{board}/variant.h`
- Rebuild and reflash after any changes

### No serial output?
- Verify baud rate is 921600
- Check cable connection
- Try: `cat /dev/ttyUSB0` at 921600 baud

## Common Commands

```bash
make heltec              # Configure for Heltec
make tbeam               # Configure for T-Beam
make build               # Compile
make flash               # Flash to device
make monitor             # View serial output
make flash-monitor       # Flash and monitor
make clean               # Remove build files
make menuconfig          # Advanced configuration
```

## Next Steps

1. Flash the firmware to your device
2. Monitor the serial output to confirm it boots
3. Check if the blue display shows status
4. Add more mesh nodes to form a network
5. Extend the radio driver for full SX127x support

## Architecture Overview

```
┌─────────────────────────────────────────┐
│         ESP32 (main.cpp)                │
│  - Initializes display, radio, tasks    │
│  - Manages mesh networking              │
│  - Coordinates with components          │
└──────────┬──────────────────────────────┘
           │
    ┌──────┴──────┬─────────────────┐
    │             │                 │
┌───▼────┐   ┌────▼────┐    ┌──────▼──────┐
│ Display │   │  Radio  │    │  GPIO/LED   │
│ (OLED)  │   │(LoRa)   │    │  (Blink)    │
└────────┘    └─────────┘    └─────────────┘
```

## Hardware Pin Reference

### Heltec
- OLED: I2C SDA=GPIO4, SCL=GPIO15
- LoRa: SPI MOSI=GPIO10, MISO=GPIO9, CLK=GPIO8, CS=GPIO18
- LoRa Control: IRQ=GPIO26, RST=GPIO14
- LED: GPIO25

### T-Beam  
- OLED: I2C SDA=GPIO21, SCL=GPIO22
- LoRa: SPI MOSI=GPIO23, MISO=GPIO19, CLK=GPIO18, CS=GPIO5
- LoRa Control: IRQ=GPIO26, RST=GPIO14
- GPS: UART1 RX=GPIO12, TX=GPIO34
- LED: GPIO25
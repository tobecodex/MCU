# Meshtastic ESP32 Node

A minimal Meshtastic mesh networking node implementation for ESP32-based boards with LoRa and OLED display support.

## Supported Hardware

- **Heltec WiFi LoRa 32** - Integrated OLED display and SX127x LoRa radio
- **LilyGO T-Beam** - GPS module with optional OLED display and SX127x LoRa radio

### Hardware Detection

This project was initialized for a board with:
- **Silicon Labs CP210x UART Bridge** (USB serial interface)
- **Blue OLED Display** (SSD1306 128x64)
- **SX127x LoRa Radio** (868/915/433 MHz capable)

## Features

- **OLED Display** - Real-time status and message display
- **LoRa Radio** - Long-range mesh communication
- **Serial Output** - Detailed logging of all operations
- **Message Monitoring** - View incoming mesh packets in real-time
- **Low Power** - Tickless idle and CPU frequency scaling

## Building

### Prerequisites
- ESP-IDF v5.0+ installed and configured
- `idf.py` in PATH
- CH340/CP210x USB serial driver (usually included in modern systems)

### Setup for Heltec Board
```bash
make heltec
make build
make flash PORT=/dev/ttyUSB0
```

### Setup for T-Beam Board
```bash
make tbeam
make build
make flash PORT=/dev/ttyUSB0
```

## Development Workflow

### Build Targets
```bash
make build           # Compile firmware
make flash           # Flash to device
make monitor         # View serial output (921600 baud)
make flash-monitor   # Flash and monitor together
make clean           # Clean build artifacts
```

### Viewing Messages

The application logs all incoming mesh packets to the serial console:

```
I (12345) Radio: RX: from=0xdeadbeef to=0xffffffff len=15 rssi=-95 snr=8
I (12346) Radio: Received mesh packet from node 0xdeadbeef
I (12347) Main: Node is alive - ready to relay messages
```

The OLED display also shows:
- Boot status
- Network status (RX listening, active relay)
- Signal strength (RSSI)
- Recent messages

### Configuration

Edit `sdkconfig.heltec` or `sdkconfig.tbeam` to customize:
- Baud rate (default 921600)
- Radio frequency (default 915 MHz for North America)
- Display options
- Power management settings

Apply configuration changes with:
```bash
make menuconfig
```

## Architecture

```
main/
  ├── main.cpp              - Application entry point
  ├── variant/
  │   ├── heltec/variant.h  - Heltec pin definitions
  │   └── tbeam/variant.h   - T-Beam pin definitions
  └── CMakeLists.txt

components/
  ├── display/              - SSD1306 OLED driver
  │   ├── display.h
  │   ├── display.cpp
  │   └── component.mk
  └── radio/                - SX127x LoRa driver
      ├── radio.h
      ├── radio.cpp
      └── component.mk
```

## Serial Output

The application provides detailed serial logging at 921600 baud:

```
I (0) cpu_start: ESP-IDF v5.0 on ESP32
I (123) Main: ========================================
I (124) Main: Meshtastic Node - Heltec
I (125) Main: ========================================
I (234) Main: Board: Heltec
I (235) Main: GPIO Config:
I (236) Main:   I2C SDA=4 SCL=15
I (237) Main:   LoRa CS=18 IRQ=26 RST=14
I (238) Main:   LED=25
I (500) Display: Initializing I2C for OLED display
I (600) Radio: Initializing SX127x LoRa radio
I (601) Radio: Frequency: 915.0 MHz
I (602) Radio: TX Power: 17 dBm
I (700) Main: MAC: aa:bb:cc:dd:ee:ff
I (701) Main: Free heap: 123456 bytes
I (800) Main: Meshtastic node ready - waiting for mesh messages...
```

When messages are received:
```
I (5432) Radio: RX: from=0x12345678 to=0xffffffff len=32 rssi=-105 snr=5
I (5433) Display: Display: Message from 0x12345678
```

## Next Steps

1. **Complete Radio Driver** - Implement SX127x SPI communication and packet handling
2. **Add Mesh Routing** - Implement Meshtastic protocol message routing
3. **Extend Display** - Add graphical display of signal strength, node info, and message history
4. **GPS Integration** - Add GPS location tracking (T-Beam only)
5. **Configuration UI** - Add web interface or mobile app support

## Troubleshooting

### Serial Port Not Found
```bash
# Check available ports
ls -la /dev/tty*
# Set PORT environment variable
make flash PORT=/dev/ttyACM0
```

### Display Not Appearing
Check I2C pins match your board variant:
- Heltec: SDA=GPIO4, SCL=GPIO15
- T-Beam: SDA=GPIO21, SCL=GPIO22

Edit `main/variant/{board}/variant.h` and rebuild.

### Radio Not Initializing
Verify SPI pins and CS/RST/IRQ GPIO assignments in variant header files.

## License

Meshtastic project - see https://meshtastic.org/
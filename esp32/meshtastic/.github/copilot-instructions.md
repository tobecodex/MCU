# Copilot Instructions for Meshtastic ESP32 Project

> **Inherits from**: `../.github/copilot-instructions.md` (ESP32 development patterns)

## Project Goal
**Build a simple Meshtastic mesh repeater node** - A minimal working device that can receive and relay LoRa mesh packets. This is a learning project focusing on fundamentals before adding custom features.

**Current Status:** Early development - basic infrastructure works (display, tasks) but radio communication needs implementation.

**Next Priority:** Get LoRa radio receiving and transmitting packets, then implement basic message relay logic.

## Project Overview
This is a Meshtastic ESP32 firmware project that implements mesh networking capabilities for LoRa devices. Uses ESP-IDF framework with Makefile-based toolchain.

## Meshtastic-Specific Architecture

### Core Mesh Components
- `main/mesh/` - Mesh networking protocol implementation
- `main/radio/` - LoRa radio driver and communication layer  
- `main/gps/` - GPS module integration and location services
- `main/display/` - OLED/LCD display management
- `main/power/` - Battery and power management specific to mesh devices
- `data/` - SPIFFS data (web interface, configuration files)

### Hardware Variant Management
- Hardware-specific configurations in `sdkconfig.{variant}` files
- Pin definitions in `main/variant/{hardware}/variant.h`
- Switch variants: `cp sdkconfig.tbeam sdkconfig`
- Common variants: T-Beam, Heltec, T-Echo, RAK4631

### Testing Strategy
- Use ESP-IDF's `unity` test framework in `test/` directory
- Run tests with `idf.py build flash monitor -p PORT`
- Hardware-specific tests require actual devices
- Use CMock for mocking hardware interfaces

## Meshtastic-Specific Patterns

### LoRa Radio Integration
- SX127x/SX126x chip communication via SPI
- Interrupt handling for packet reception
- Frequency and power management based on region
- CSMA/CA for collision avoidance

### GPS Integration
- UART communication with GPS module
- NMEA sentence parsing
- Position validation and accuracy filtering
- Power management for battery conservation

### Web Interface
- SPIFFS filesystem for static web content
- AsyncWebServer for HTTP endpoints
- WebSocket for real-time updates
- JSON API for configuration and status

### Bluetooth Integration
- ESP32 Bluetooth Classic for mobile app communication
- Custom service UUIDs for Meshtastic protocol
- Handle connection/disconnection events

## Meshtastic Protocol Patterns

### Singleton Services
Most hardware interfaces use singleton pattern:
```cpp
RadioInterface& radio = RadioInterface::getInstance();
GPSStatus* gps = GPSStatus::getInstance();
```

### Message Protocol
- Protobuf for inter-node communication
- Message routing based on node IDs
- Hop count and duplicate detection
- Store-and-forward for offline nodes

### Power Management
- Deep sleep between transmissions
- CPU frequency scaling based on activity
- Peripheral power control (GPS, display)
- Battery voltage monitoring

## Key Files to Understand
- `main/main.cpp` - Application lifecycle and task coordination
- `main/mesh/Router.cpp` - Message routing logic
- `main/RadioInterface.cpp` - LoRa hardware abstraction
- `sdkconfig` - ESP-IDF build configuration
- `CMakeLists.txt` - Build system configuration
- `main/variant/*/variant.h` - Hardware-specific pin definitions

## Debugging Tips
- Use `ESP_LOGD` for debug output (disabled in release builds)
- Serial monitor at 921600 baud for development: `idf.py monitor -b 921600`
- Hardware debugger support via JTAG with OpenOCD
- Memory usage monitoring with `esp_get_free_heap_size()`
- Use `idf.py size` to analyze binary size and memory usage
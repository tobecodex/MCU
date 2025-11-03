# Embedded Development Projects

Multi-platform embedded development workspace with hierarchical AI agent instructions.

## Project Structure

```
src/
├── .github/
│   └── copilot-instructions.md    # General development patterns
├── esp32/                          # ESP32 projects (ESP-IDF)
│   ├── .github/
│   │   └── copilot-instructions.md # ESP32-specific patterns
│   └── meshtastic/                 # Meshtastic mesh node
├── arduino/                        # Arduino/PlatformIO projects
│   └── .github/
│       └── copilot-instructions.md # Arduino-specific patterns
├── pico/                           # Raspberry Pi Pico projects
│   └── .github/
│       └── copilot-instructions.md # Pico-specific patterns
└── microbit/                       # BBC micro:bit projects
```

## Platforms

### ESP32
- **Framework**: ESP-IDF 6.0
- **Toolchain**: `idf.py` (Makefile-based)
- **Projects**: Meshtastic LoRa mesh networking node

### Arduino
- **Framework**: Arduino/PlatformIO
- **Toolchain**: `pio` or `arduino-cli`
- **Boards**: Various (Uno, Nano, ESP32, Adafruit Feather)

### Raspberry Pi Pico
- **Framework**: Pico SDK (C/C++) or MicroPython
- **Toolchain**: CMake or `mpremote`
- **Hardware**: RP2040 dual-core, PIO state machines

### BBC micro:bit
- **Framework**: MicroPython or C/C++ (CODAL)
- **Hardware**: nRF52833, LED matrix, sensors

## AI Agent Instructions

This repository uses hierarchical `.github/copilot-instructions.md` files:

1. **Root level** (`src/.github/`) - General development practices
2. **Platform level** (`esp32/.github/`, etc.) - Platform-specific patterns
3. **Project level** (`meshtastic/.github/`) - Project-specific details

Each level inherits from its parent, building specificity from general to detailed.

## Getting Started

### ESP32 Projects
```bash
cd esp32/meshtastic
. ~/.esp/v6.0/esp-idf/export.sh
make build
make flash PORT=/dev/ttyUSB0
```

### Arduino Projects
```bash
cd arduino
pio run
pio run -t upload
```

### Pico Projects
```bash
cd pico/project
cmake -B build
cmake --build build
# Copy build/*.uf2 to Pico in BOOTSEL mode
```

## License

Individual projects may have their own licenses. Check project directories for details.

# Copilot Instructions for Raspberry Pi Pico Development

> **Inherits from**: `../../.github/copilot-instructions.md` (General development practices)

## Toolchain
- Use Pico SDK (C/C++) or MicroPython/CircuitPython
- Check for `pico-sdk` installation before proceeding
- CMake-based builds for C/C++ projects
- Python virtual environment for MicroPython development

## C/C++ SDK Build
- Commands: `cmake ..`, `make`, `make -j$(nproc)`
- Output: `*.uf2` files for direct USB upload
- Flash: Copy `.uf2` to mounted Pico drive (BOOTSEL mode)
- Serial: `/dev/ttyACM0` at 115200 baud

## MicroPython
- Use `mpremote` or `ampy` for file transfer
- REPL access: `mpremote connect /dev/ttyACM0`
- Upload: `mpremote fs cp file.py :`
- Virtual environment recommended for tools

## Pico Patterns
- Dual-core RP2040 (use both cores for performance)
- PIO state machines for precise timing/protocols
- Flash: 2MB, RAM: 264KB
- GPIO: 26 multi-function pins, 3.3V logic

## Hardware
- RP2040 chip, USB-C connector
- Reset: Hold BOOTSEL + press reset, or power cycle
- Debug: SWD via Picoprobe or another Pico

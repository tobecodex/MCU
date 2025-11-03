# Copilot Instructions for Arduino Development

> **Inherits from**: `../../.github/copilot-instructions.md` (General development practices)

## Toolchain
- Use PlatformIO or Arduino CLI (check which is installed first)
- PlatformIO commands: `pio run`, `pio run -t upload`, `pio device monitor`
- Arduino CLI: `arduino-cli compile`, `arduino-cli upload`

## Build & Upload
- Check `platformio.ini` for board config and libraries
- Verify builds succeed before upload
- Use correct serial port (usually `/dev/ttyUSB*` or `/dev/ttyACM*`)

## Arduino Patterns
- `setup()` for initialization, `loop()` for main execution
- Use `Serial.begin(115200)` for debugging
- Prefer non-blocking code (avoid `delay()` in complex logic)
- Libraries in `lib/` or managed by PlatformIO

## Hardware
- Check board type and voltage levels (3.3V vs 5V)
- Pin definitions in board variant files
- Common boards: Uno, Nano, Mega, ESP32, ESP8266, Adafruit Feather

# Copilot Instructions for ESP32 Development

> **Inherits from**: `../../.github/copilot-instructions.md` (General development practices)

## ESP-IDF Installation

**ESP-IDF Location:** `~/.esp/v6.0/esp-idf`

## Build System

**CRITICAL: ALWAYS use the project Makefile, NEVER run `idf.py` directly.**

The Makefile handles ESP-IDF environment setup automatically.

Standard commands:
- `make build` - Build firmware
- `make flash` - Flash to device
- `make monitor` - Monitor serial output
- `make clean` - Clean build artifacts

## ESP-IDF Patterns

### Build & Config
- `sdkconfig` for build config, NVS for runtime
- `main/` for app code, `components/` for reusable modules

### Memory & Tasks
- RAII, prefer stack over heap, check malloc returns
- `esp_heap_caps_malloc()` for DMA/PSRAM
- FreeRTOS: `xTaskCreate(fn, name, stack, NULL, priority, &handle)`
- Stack: 4096+ for complex tasks, priorities 0-25

### Logging & Errors
- `ESP_LOG[I|W|E|D]()` macros
- Check ESP-IDF return values, use `assert()` for logic errors

### Hardware
- SPI: `spi_master`, I2C: `i2c` driver, UART: `uart_driver_install()`
- GPIO: `gpio_config()`

### Power & Debug
- Sleep: `esp_deep_sleep_start()`, `esp_light_sleep_start()`
- Monitor at 921600 baud

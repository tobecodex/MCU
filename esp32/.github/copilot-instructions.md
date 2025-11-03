# Copilot Instructions for ESP32 Development

> **Inherits from**: `../../.github/copilot-instructions.md` (General development practices)

## Toolchain
- Use `idf.py` (not PlatformIO), Makefile builds preferred
- Check tool installation before installing (`which`, `command -v`)
- Verify builds succeed before flash (check for `*.bin` files)
- Use background tasks for long operations (`isBackground=true`)

## ESP-IDF Patterns

### Build & Config
- Commands: `idf.py build/flash/monitor/menuconfig`
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
- Monitor: `idf.py monitor -b 921600`
- Memory: `esp_get_free_heap_size()`, size: `idf.py size`
Adafruit boards and PlatformIO

Common board IDs (as of PlatformIO board registry):

- adafruit_feather_m0 — Adafruit Feather M0
- adafruit_feather_nrf52840 — Adafruit Feather nRF52840
- adafruit_metro_m4 — Adafruit Metro M4 (SAMD51)

Detecting a connected board

- On Linux, use `lsusb` to see USB devices. Some boards show up with Adafruit or Atmel vendor IDs.
- Use `platformio device list` to see serial ports and detect which one is your board.
- To find the correct PlatformIO board ID, search the PlatformIO boards registry: https://platformio.org/boards

Adding new boards

1. Find the board ID on PlatformIO.
2. Copy an existing `[env:*]` block in `platformio.ini` and change `board = <your_board_id>`.
3. Run `platformio run` to fetch packages and build.

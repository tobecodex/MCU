# Arduino PlatformIO Workspace

This workspace provides a base PlatformIO project for Adafruit boards.

Getting started

1. Install PlatformIO (VS Code extension or CLI): https://platformio.org
2. Open this folder (`/home/tobe/src/arduino`) in VS Code.
3. Run PlatformIO: Build with the default env or change `default_envs` in `platformio.ini`.

Common commands

- Build: platformio run
- Upload: platformio run -t upload
- Monitor serial: platformio device monitor

Files

- `platformio.ini` — Project configuration with example Adafruit environments.
- `src/main.cpp` — Example Arduino sketch (PlatformIO-style).
- `adafruit_boards.md` — Notes for finding board IDs and mapping to PlatformIO.
- `.vscode/tasks.json` — VS Code tasks for build/upload/monitor (if using VS Code).

Notes

This scaffold targets common Adafruit Feather and Metro boards. If you use a different board, add an environment to `platformio.ini` and set the correct `board` ID as documented by PlatformIO.

## Wiring: WS2812B (NeoPixel) LED strip

The example in `src/main.cpp` targets a WS2812B/NeoPixel strip:

- Type: WS2812B (800 kHz, GRB)
- Count: 300 LEDs (`NUMPIXELS 300`)
- Data pin: Arduino D5 (`#define PIN 5`)
- Default board: Adafruit ItsyBitsy M0 Express (3.3V logic)

Follow these steps to wire the strip safely:

1) Power the strip
- Use a separate, regulated 5V supply for the LEDs. Do NOT power the strip from the board’s 5V/USB pin.
- Worst case current: up to ~60 mA per LED at full white → 300 × 0.06 A ≈ 18 A. Your actual current will be much lower at the configured brightness (`setBrightness(10)`), but size the supply generously (e.g., 10–20 A for the full strip) or limit brightness/LED count.
- Place a large electrolytic capacitor across the strip power at the first LED: 1000 µF (or larger), ≥6.3 V. Polarity: + to +5V, − to GND.
- For long strips, inject power at both ends and optionally every 50–100 LEDs to reduce voltage drop. Use adequate wire gauge.

2) Common ground
- Connect the power supply GND to the LED strip GND.
- Connect the board GND to the LED strip GND. All grounds must be common.

3) Data line (D5 → DIN)
- Connect the board’s D5 pin to the strip’s DIN through a 220–470 Ω resistor (series resistor near the strip input).
- Keep the data wire short and routed alongside a ground wire if possible.

4) Logic level (3.3V MCU → 5V strip)
- The ItsyBitsy M0 is 3.3V logic, while WS2812B expects a logic-high near 5V when powered at 5V. Many strips work at 3.3V data if the wiring is short and the first LED is forgiving, but the robust solution is to shift the data to 5V using a 74AHCT125/74HCT245 (AHCT/HCT family). Wire the shifter: 3.3V side from D5, 5V side to DIN; power the shifter from the 5V LED supply; connect grounds.
- Alternative: run the strip a bit under 5V (e.g., 4.5–4.7V via a diode drop), which can make 3.3V logic high more reliable. Still ensure grounds are common.

5) Double-check polarity on the strip
- Identify the strip input end (arrows toward the strip). Connect DIN, +5V, and GND at that end.
- Never reverse +5V and GND.

6) First power-on checklist
- Set brightness low first (already `setBrightness(10)` in code).
- Power the LED supply before or at the same time as the microcontroller. Avoid back-powering your computer via USB through the LED supply; if in doubt, power the MCU from USB and the LEDs from the external 5V with shared ground only.

Uploading and testing

- Ensure `default_envs = itsybitsy_m0_express` in `platformio.ini` matches your board, or select another env.
- Build and upload, then your strip should show a single pixel bouncing along the strip. At very low brightness it may be dim in bright rooms; test in a dim room or temporarily increase brightness after confirming your power can handle it.

Optional: serial monitor at 115200 baud to see the current pixel index.

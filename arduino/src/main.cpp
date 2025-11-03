#include <Adafruit_NeoPixel.h>
#include <math.h>

// Enable USB safe mode for low current draw
#define USB_SAFE_MODE 1

// WS2812B LED strip
#define NUMPIXELS 64
#define PIN       5  // Data pin connected to the strip

// Button for pattern switching (use a momentary switch to GND)
#define BUTTON_PIN 9           // D9 on ItsyBitsy M0 (label "9!")
#define DEBOUNCE_MS 30         // Debounce time

// Animation speed (lower is faster)
#define BOUNCE_DELAY_MS 5
// Plasma tuning
#define PLASMA_CONTRAST 220   // 128 = no change; >128 increases contrast
#define PLASMA_SPEED 3        // Animation speed for plasma (base step)
#define PLASMA_FRAME_DIV 10   // Divide plasma speed by this factor (~10x slower)

// Game of Life tuning
#define LIFE_FRAME_DIV 10     // Compute next generation every N frames (~10x slower)
// Comet advances only every N frames (higher is slower)
#define COMET_STEP_FRAMES 12

// 8x8 Grid configuration
#define GRID_WIDTH 8
#define GRID_HEIGHT 8
#define GRID_SERPENTINE true  // true if rows alternate direction (zig-zag wiring)

// USB safe mode settings
#if USB_SAFE_MODE
#define SAFE_BRIGHTNESS 30  // Increased for better visibility on grid
// Allow full 8x8 grid (64 pixels) for game/test patterns
#define SAFE_MAX_ON_PIXELS 64
#else
#define SAFE_BRIGHTNESS 50
#define SAFE_MAX_ON_PIXELS NUMPIXELS
#endif

// Create NeoPixel object
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Maintain a per-frame count of lit pixels to cap current draw
static uint16_t g_onCount = 0;

void safeClearFrame() {
  strip.clear();
  g_onCount = 0;
}

void safeSetPixel(int idx, uint32_t color) {
  if (idx < 0 || idx >= NUMPIXELS) return;
  if (color == 0) { strip.setPixelColor(idx, 0); return; }
  uint32_t prev = strip.getPixelColor(idx);
  if (prev == 0) {
    if (g_onCount >= SAFE_MAX_ON_PIXELS) return; // budget exhausted
    g_onCount++;
  }
  strip.setPixelColor(idx, color);
}

// Map 8x8 grid coordinates to linear LED index
// Assumes first LED is at (0,0) top-left
int gridToIndex(int x, int y) {
  if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) return -1;
  
  if (GRID_SERPENTINE) {
    // Zig-zag: even rows left-to-right, odd rows right-to-left
    if (y % 2 == 0) {
      return y * GRID_WIDTH + x;
    } else {
      return y * GRID_WIDTH + (GRID_WIDTH - 1 - x);
    }
  } else {
    // All rows left-to-right
    return y * GRID_WIDTH + x;
  }
}

void setGridPixel(int x, int y, uint32_t color) {
  int idx = gridToIndex(x, y);
  if (idx >= 0) safeSetPixel(idx, color);
}

// ---------------- Pattern and Input State ----------------
enum Pattern : uint8_t {
  PATTERN_GRID_TEST = 0,
  PATTERN_FLOWER = 1,
  PATTERN_WALKING = 2,
  PATTERN_BLINKING_EYE = 3,
  PATTERN_ROCKET = 4,
  PATTERN_GAME_OF_LIFE = 5,
   PATTERN_PLASMA = 6,
   PATTERN_BOUNCE = 7,
   PATTERN_HOP_CHASE = 8,
   PATTERN_COMET_TAIL = 9,
   PATTERN_LARSON = 10,
   PATTERN_TWINKLE = 11,
   PATTERN_DOTS_CHASE = 12,
   PATTERN_SPARKLE = 13,
   PATTERN_PINGPONG = 14,
   PATTERN_BREATH = 15,
  PATTERN_COUNT
};

volatile Pattern currentPattern = PATTERN_GRID_TEST;

// Consolidated animation state
#define MAX_TWINKLES 8
struct AnimationState {
  // Bounce/movement patterns
  struct { int pos, dir; } bounce, comet, larson, ping;
  struct { int pos; } hop;
  uint8_t cometFrame;
  
  // Twinkle
  struct { int pos[MAX_TWINKLES]; uint8_t level[MAX_TWINKLES]; bool active[MAX_TWINKLES]; } twinkle;
  
  // Dots chase
  struct { int offset, spacing; } dots;
  
  // Sparkle
  struct { bool on; int pos; uint8_t life; } sparkle;
  
  // Breath
  struct { uint8_t level; int dir; } breath;
  
  // Button debounce
  struct { int lastReading, stableState; unsigned long lastDebounceTime; } button;
} anim = {
  .bounce = {0, 1},
  .comet = {0, 1},
  .larson = {0, 1},
  .ping = {0, 1},
  .hop = {0},
  .cometFrame = 0,
  .twinkle = {{}, {}, {}},
  .dots = {0, 20},
  .sparkle = {false, 0, 0},
  .breath = {0, 4},
  .button = {HIGH, HIGH, 0}
};

// Constants
static const int hopStep = 10;
static const uint8_t twFade = 20;
static const int breathMin = 10;
static const int breathMax = 255;

uint32_t redLevel(uint8_t level) {
  return strip.Color(level, 0, 0);
}

const char* patternName(Pattern p) {
  switch (p) {
    case PATTERN_GRID_TEST: return "Grid Test";
    case PATTERN_FLOWER: return "Flower";
    case PATTERN_WALKING: return "Walking";
    case PATTERN_BLINKING_EYE: return "Blinking Eye";
    case PATTERN_ROCKET: return "Rocket";
    case PATTERN_GAME_OF_LIFE: return "Game of Life";
     case PATTERN_PLASMA: return "Plasma";
    case PATTERN_BOUNCE: return "Bounce";
    case PATTERN_HOP_CHASE: return "Hop-Chase";
    case PATTERN_COMET_TAIL: return "Comet Tail";
    case PATTERN_LARSON: return "Larson";
    case PATTERN_TWINKLE: return "Twinkle";
    case PATTERN_DOTS_CHASE: return "Dots Chase";
    case PATTERN_SPARKLE: return "Sparkle";
    case PATTERN_PINGPONG: return "Ping-Pong";
    case PATTERN_BREATH: return "Breath";
    default: return "Unknown";
  }
}

void nextPattern() {
  currentPattern = (Pattern)((currentPattern + 1) % PATTERN_COUNT);
  // Reset per-pattern state for clean transitions
  anim.bounce = {0, 1};
  anim.hop.pos = 0;
  anim.comet = {0, 1};
  anim.cometFrame = 0;
  anim.larson = {0, 1};
  for (int i = 0; i < MAX_TWINKLES; ++i) {
    anim.twinkle.active[i] = false;
    anim.twinkle.level[i] = 0;
    anim.twinkle.pos[i] = -1;
  }
  anim.dots = {0, 20};
  anim.sparkle = {false, 0, 0};
  anim.ping = {0, 1};
  anim.breath.level = 0;
  anim.breath.dir = abs(anim.breath.dir);
}

void readButtonAndMaybeAdvance() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading != anim.button.lastReading) {
    anim.button.lastDebounceTime = millis();
  }
  anim.button.lastReading = reading;
  if ((millis() - anim.button.lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != anim.button.stableState) {
      anim.button.stableState = reading;
      // Detect press: HIGH -> LOW (with pullup)
      if (anim.button.stableState == LOW) {
        nextPattern();
        Serial.print("Pattern ");
        Serial.print((int)currentPattern);
        Serial.print(": ");
        Serial.println(patternName(currentPattern));
      }
    }
  }
}

// ---------------- Pattern Drawers ----------------

// Generic pixel art renderer - draws an 8x8 indexed pixel art with palette
void renderPixelArt(const uint8_t art[8][8], const uint32_t* palette, int paletteSize) {
  safeClearFrame();
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      uint8_t idx = art[y][x];
      if (idx > 0 && idx < paletteSize) {
        setGridPixel(x, y, palette[idx]);
      }
    }
  }
  strip.show();
}

// Unified bouncer for strip patterns - handles bounce, larson, pingpong, comet
// tailLen: 0=single pixel, >0 adds tail
// bidirectional: false=tail behind, true=tail both sides (Larson)
// tailBrightness: array of brightness levels for tail segments
void drawBouncer(int& pos, int& dir, int tailLen, bool bidirectional, const uint8_t* tailBrightness) {
  safeClearFrame();
  
  // Draw head
  safeSetPixel(pos, redLevel(255));
  
  // Draw tail(s)
  for (int t = 1; t <= tailLen; ++t) {
    if (t >= 4) break; // Max tail length
    uint8_t brightness = tailBrightness ? tailBrightness[t-1] : (255 >> t);
    
    if (bidirectional) {
      // Larson-style: tails on both sides
      int back = pos - t;
      int fwd = pos + t;
      if (back >= 0) safeSetPixel(back, redLevel(brightness));
      if (fwd < NUMPIXELS) safeSetPixel(fwd, redLevel(brightness));
    } else {
      // Comet-style: tail behind
      int idx = pos - t * dir;
      if (idx < 0) idx += NUMPIXELS;
      if (idx >= NUMPIXELS) idx -= NUMPIXELS;
      safeSetPixel(idx, redLevel(brightness));
    }
  }
  
  strip.show();
  
  // Move and bounce
  pos += dir;
  if (pos <= 0 || pos >= NUMPIXELS - 1) dir = -dir;
}

void drawGridTest() {
  safeClearFrame();
  static uint8_t hueShift = 0;
  
  // Rainbow gradient across the 8x8 grid
  for (int y = 0; y < GRID_HEIGHT; y++) {
    for (int x = 0; x < GRID_WIDTH; x++) {
      // Calculate hue based on position and time
      uint8_t hue = hueShift + (x * 32) + (y * 32);
      
      // Convert HSV to RGB (simple approximation)
      uint8_t third = hue / 85;
      uint8_t offset = hue % 85;
      uint8_t r = 0, g = 0, b = 0;
      
      if (third == 0) {
        r = 255 - offset * 3;
        g = offset * 3;
        b = 0;
      } else if (third == 1) {
        r = 0;
        g = 255 - offset * 3;
        b = offset * 3;
      } else {
        r = offset * 3;
        g = 0;
        b = 255 - offset * 3;
      }
      
      setGridPixel(x, y, strip.Color(r, g, b));
    }
  }
  strip.show();
  hueShift += 2; // Slowly rotate colors
}

void drawFlower() {
  // 8x8 flower pixel art (flipped for bottom-right origin)
  // 0=black, 1=yellow/center, 2=red/petals, 3=green/stem, 4=green/leaves
  static const uint8_t flower[8][8] = {
    {0, 0, 0, 3, 3, 0, 0, 0},
    {0, 0, 4, 3, 3, 4, 0, 0},
    {0, 0, 2, 3, 3, 2, 0, 0},
    {0, 2, 2, 1, 1, 2, 2, 0},
    {0, 2, 1, 1, 1, 1, 2, 0},
    {0, 2, 2, 1, 1, 2, 2, 0},
    {0, 0, 2, 2, 2, 2, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}
  };
  
  // Color palette
  static const uint32_t colors[5] = {
    strip.Color(0, 0, 0),       // 0: black (off)
    strip.Color(255, 255, 0),   // 1: yellow (center)
    strip.Color(255, 0, 100),   // 2: pink/red (petals)
    strip.Color(0, 180, 0),     // 3: green (stem)
    strip.Color(50, 255, 50)    // 4: bright green (leaves)
  };
  
  renderPixelArt(flower, colors, 5);
}

void drawWalking() {
  // Smooth side-view walk at 32x32 → 8x8 with better anti-aliasing
  safeClearFrame();

  const int HRW = 32, HRH = 32; // Higher res for smoother AA
  static uint8_t hr[HRH][HRW];
  static float phase = 0.0f;
  
  // Clear HR buffer
  for (int y = 0; y < HRH; ++y) 
    for (int x = 0; x < HRW; ++x) 
      hr[y][x] = 0;

  // Anti-aliased line drawing (Bresenham + Wu)
  auto plot = [&](int x, int y, uint8_t v) {
    if ((unsigned)x < (unsigned)HRW && (unsigned)y < (unsigned)HRH) {
      int nv = hr[y][x] + v;
      hr[y][x] = (nv > 255) ? 255 : nv;
    }
  };
  
  auto drawLine = [&](float x0, float y0, float x1, float y1, uint8_t brightness) {
    // Wu's algorithm for smooth anti-aliased lines
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.1f) return;
    dx /= len; dy /= len;
    
    int steps = (int)(len + 0.5f);
    for (int i = 0; i <= steps; ++i) {
      float fx = x0 + dx * i;
      float fy = y0 + dy * i;
      int ix = (int)fx, iy = (int)fy;
      float fracX = fx - ix, fracY = fy - iy;
      
      // 4-way subpixel distribution
      uint8_t b00 = (uint8_t)(brightness * (1-fracX) * (1-fracY));
      uint8_t b10 = (uint8_t)(brightness * fracX * (1-fracY));
      uint8_t b01 = (uint8_t)(brightness * (1-fracX) * fracY);
      uint8_t b11 = (uint8_t)(brightness * fracX * fracY);
      
      plot(ix, iy, b00);
      plot(ix+1, iy, b10);
      plot(ix, iy+1, b01);
      plot(ix+1, iy+1, b11);
    }
  };
  
  auto drawCircle = [&](float cx, float cy, float r, uint8_t brightness) {
    int r2 = (int)(r*r + 0.5f);
    int ir = (int)(r + 1.0f);
    for (int dy = -ir; dy <= ir; ++dy) {
      for (int dx = -ir; dx <= ir; ++dx) {
        float d2 = dx*dx + dy*dy;
        if (d2 <= r2) {
          float alpha = 1.0f;
          if (d2 > (r-1)*(r-1)) {
            // Soften edge
            alpha = r - sqrtf(d2);
            if (alpha < 0) alpha = 0;
            if (alpha > 1) alpha = 1;
          }
          plot((int)(cx + dx), (int)(cy + dy), (uint8_t)(brightness * alpha));
        }
      }
    }
  };

  // Body proportions (scaled for 32x32)
  const float groundY = 28.0f;
  const float hipX = 16.0f, hipY = 18.0f;
  const float neckX = 16.0f, neckY = 10.0f;
  const float headR = 6.0f;
  
  const float thighLen = 9.0f;
  const float shinLen = 9.0f;
  const float upperArmLen = 7.0f;
  const float forearmLen = 6.0f;

  // Smoother walking cycle - use both sin and cos for natural motion
  float t = phase;
  
  // Left leg phase (0 to 2π for full stride)
  float legPhaseL = t;
  float legPhaseR = t + (float)M_PI;
  
  // Thigh angle: forward during swing, back during stance
  float thighAngleL = 0.35f * sinf(legPhaseL);
  float thighAngleR = 0.35f * sinf(legPhaseR);
  
  // Knee bend: more bend during swing phase
  float kneeExtraL = (sinf(legPhaseL) > 0) ? 0.3f * sinf(legPhaseL * 2.0f) : 0.0f;
  float kneeExtraR = (sinf(legPhaseR) > 0) ? 0.3f * sinf(legPhaseR * 2.0f) : 0.0f;
  float shinAngleL = thighAngleL + 0.4f + kneeExtraL;
  float shinAngleR = thighAngleR + 0.4f + kneeExtraR;
  
  // Compute leg joints
  float kneeXL = hipX + thighLen * sinf(thighAngleL);
  float kneeYL = hipY + thighLen * cosf(thighAngleL);
  float footXL = kneeXL + shinLen * sinf(shinAngleL);
  float footYL = kneeYL + shinLen * cosf(shinAngleL);
  
  float kneeXR = hipX + thighLen * sinf(thighAngleR);
  float kneeYR = hipY + thighLen * cosf(thighAngleR);
  float footXR = kneeXR + shinLen * sinf(shinAngleR);
  float footYR = kneeYR + shinLen * cosf(shinAngleR);
  
  // Foot planting - stick to ground during stance
  if (footYL > groundY) footYL = groundY;
  if (footYR > groundY) footYR = groundY;
  
  // Arm swing - opposite to legs, smaller range
  float armAngleL = -0.25f * sinf(legPhaseL);
  float armAngleR = -0.25f * sinf(legPhaseR);
  float forearmAngleL = armAngleL - 0.15f * sinf(legPhaseL + 1.0f);
  float forearmAngleR = armAngleR - 0.15f * sinf(legPhaseR + 1.0f);
  
  // Compute arm joints
  float elbowXL = neckX + upperArmLen * sinf(armAngleL);
  float elbowYL = neckY + upperArmLen * cosf(armAngleL);
  float handXL = elbowXL + forearmLen * sinf(forearmAngleL);
  float handYL = elbowYL + forearmLen * cosf(forearmAngleL);
  
  float elbowXR = neckX + upperArmLen * sinf(armAngleR);
  float elbowYR = neckY + upperArmLen * cosf(armAngleR);
  float handXR = elbowXR + forearmLen * sinf(forearmAngleR);
  float handYR = elbowYR + forearmLen * cosf(forearmAngleR);

  // Draw in depth order: far limbs → torso → near limbs
  uint8_t farLimb = 100, nearLimb = 180, torso = 140, head = 160;
  
  // Far leg/arm (R)
  drawLine(hipX, hipY, kneeXR, kneeYR, farLimb);
  drawLine(kneeXR, kneeYR, footXR, footYR, farLimb);
  drawLine(neckX, neckY, elbowXR, elbowYR, farLimb);
  drawLine(elbowXR, elbowYR, handXR, handYR, farLimb);
  
  // Torso & head
  drawLine(neckX, neckY, hipX, hipY, torso);
  drawCircle(neckX, neckY - (headR * 0.7f), headR, head);
  
  // Near leg/arm (L)
  drawLine(hipX, hipY, kneeXL, kneeYL, nearLimb);
  drawLine(kneeXL, kneeYL, footXL, footYL, nearLimb);
  drawLine(neckX, neckY, elbowXL, elbowYL, nearLimb);
  drawLine(elbowXL, elbowYL, handXL, handYL, nearLimb);
  
  // Ground line
  for (int x = 0; x < HRW; ++x) 
    plot(x, (int)groundY, 20);

  // Downsample 4x4 → 1 with averaging for smooth 8x8 output
  for (int gy = 0; gy < 8; ++gy) {
    for (int gx = 0; gx < 8; ++gx) {
      int sum = 0;
      for (int dy = 0; dy < 4; ++dy) {
        for (int dx = 0; dx < 4; ++dx) {
          sum += hr[gy*4 + dy][gx*4 + dx];
        }
      }
      int avg = sum >> 4; // divide by 16
      if (avg > 0) setGridPixel(gx, gy, strip.Color(avg, avg, avg));
    }
  }
  strip.show();

  // Much slower animation
  phase += 0.04f;
  if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
}

void drawBlinkingEye() {
  static uint8_t frame = 0;
  static uint8_t frameCounter = 0;
  
  // 6-frame blinking animation
  // 0=black, 1=white (eye white), 2=red (iris)
  static const uint8_t blink[6][8][8] = {
    // Frame 0-2: Eye fully open
    {
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 1, 1, 1, 1, 1, 1, 0},
      {0, 1, 1, 2, 2, 1, 1, 0},
      {0, 1, 1, 2, 2, 1, 1, 0},
      {0, 1, 1, 1, 1, 1, 1, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0}
    },
    {
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 1, 1, 1, 1, 1, 1, 0},
      {0, 1, 1, 2, 2, 1, 1, 0},
      {0, 1, 1, 2, 2, 1, 1, 0},
      {0, 1, 1, 1, 1, 1, 1, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0}
    },
    {
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 1, 1, 1, 1, 1, 1, 0},
      {0, 1, 1, 2, 2, 1, 1, 0},
      {0, 1, 1, 2, 2, 1, 1, 0},
      {0, 1, 1, 1, 1, 1, 1, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0}
    },
    // Frame 3: Half closed
    {
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 1, 1, 2, 2, 1, 1, 0},
      {0, 1, 1, 2, 2, 1, 1, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0}
    },
    // Frame 4: Closed
    {
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0}
    },
    // Frame 5: Half open (reopening)
    {
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 1, 1, 2, 2, 1, 1, 0},
      {0, 1, 1, 2, 2, 1, 1, 0},
      {0, 0, 1, 1, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0}
    }
  };
  
  // Color palette
  static const uint32_t colors[3] = {
    strip.Color(0, 0, 0),       // 0: black (off)
    strip.Color(200, 200, 200), // 1: white (eye white)
    strip.Color(255, 0, 0)      // 2: red (iris)
  };
  
  // Draw current frame
  renderPixelArt(blink[frame], colors, 3);
  
  // Frame timing: hold open frames longer, blink quickly
  uint8_t frameDelay = 15; // Default delay
  if (frame < 3) {
    frameDelay = 20; // Hold eye open longer
  } else {
    frameDelay = 5;  // Blink quickly
  }
  
  frameCounter++;
  if (frameCounter >= frameDelay) {
    frameCounter = 0;
    frame = (frame + 1) % 6;
  }
}

void drawRocket() {
  safeClearFrame();
  static int8_t rocketY = 7; // Start at bottom
  static uint8_t frame = 0;
  static uint8_t frameCounter = 0;
  
  // Color palette
  const uint32_t white = strip.Color(200, 200, 200);  // Rocket body
  const uint32_t red = strip.Color(255, 0, 0);        // Rocket nose/window
  const uint32_t yellow = strip.Color(255, 200, 0);   // Flame bright
  const uint32_t orange = strip.Color(255, 50, 0);    // Flame medium
  const uint32_t darkOrange = strip.Color(150, 20, 0); // Flame dark
  
  // Draw rocket body (centered, 2 pixels wide)
  if (rocketY >= 0 && rocketY < 8) {
    setGridPixel(3, rocketY, white);
    setGridPixel(4, rocketY, white);
  }
  if (rocketY - 1 >= 0 && rocketY - 1 < 8) {
    setGridPixel(3, rocketY - 1, white);
    setGridPixel(4, rocketY - 1, white);
  }
  // Rocket nose (red window)
  if (rocketY - 2 >= 0 && rocketY - 2 < 8) {
    setGridPixel(3, rocketY - 2, red);
    setGridPixel(4, rocketY - 2, red);
  }
  // Rocket tip
  if (rocketY - 3 >= 0 && rocketY - 3 < 8) {
    setGridPixel(3, rocketY - 3, white);
    setGridPixel(4, rocketY - 3, white);
  }
  
  // Animated flames (flicker between frames)
  if (rocketY < 8) {
    // Frame 0 and 2: Tall flames
    if (frame == 0 || frame == 2) {
      // Bottom row - bright yellow
      if (rocketY + 1 < 8) {
        setGridPixel(3, rocketY + 1, yellow);
        setGridPixel(4, rocketY + 1, yellow);
      }
      // Second row - orange with spread
      if (rocketY + 2 < 8) {
        setGridPixel(2, rocketY + 2, orange);
        setGridPixel(3, rocketY + 2, orange);
        setGridPixel(4, rocketY + 2, orange);
        setGridPixel(5, rocketY + 2, orange);
      }
      // Third row - darker orange, wider
      if (rocketY + 3 < 8) {
        setGridPixel(2, rocketY + 3, darkOrange);
        setGridPixel(5, rocketY + 3, darkOrange);
      }
    }
    // Frame 1 and 3: Shorter flames
    else {
      // Bottom row - bright yellow
      if (rocketY + 1 < 8) {
        setGridPixel(3, rocketY + 1, yellow);
        setGridPixel(4, rocketY + 1, yellow);
      }
      // Second row - orange
      if (rocketY + 2 < 8) {
        setGridPixel(3, rocketY + 2, orange);
        setGridPixel(4, rocketY + 2, orange);
      }
    }
  }
  
  strip.show();
  
  // Animation timing
  frameCounter++;
  if (frameCounter >= 3) {
    frameCounter = 0;
    frame = (frame + 1) % 4;
    
    // Move rocket up every 4 frames
    if (frame == 0) {
      rocketY--;
      // Reset when rocket goes off screen
      if (rocketY < -5) {
        rocketY = 7; // Reset to bottom
      }
    }
  }
}

void drawGameOfLife() {
  // Ecosystem simulation on 8x8 grid with Plants, Herbivores, Omnivores, Carnivores
  // Layers:
  // - plant[y][x] = 0/1 (plants can co-exist with an animal in the same cell)
  // - animal[y][x] = 0 none, 1 herbivore, 2 omnivore, 3 carnivore
  // - energy[y][x] = energy of animal (0..255)

  static uint8_t plant[8][8];
  static uint8_t animal[8][8];
  static uint8_t energy[8][8];

  static uint8_t plantNext[8][8];
  static uint8_t animalNext[8][8];
  static uint8_t energyNext[8][8];

  static bool initialized = false;
  static uint16_t ticks = 0;     // steps evolved
  static uint8_t frameHold = 0;  // to slow down with LIFE_FRAME_DIV

  // Parameters (tunable)
  const uint8_t metaHerb = 2;   // metabolism cost per step
  const uint8_t metaOmni = 3;
  const uint8_t metaCarn = 4;

  const uint8_t gainPlant = 15; // energy gained when eating plant
  const uint8_t gainHerb  = 25; // energy gained when eating herbivore
  const uint8_t gainOmni  = 20; // energy gained when eating omnivore (by carnivore)

  const uint8_t reproThresh = 40; // energy needed to reproduce
  const uint8_t reproCost   = 20; // energy cost to parent when reproducing
  const uint8_t childInit   = 15; // initial energy for child

  const uint8_t plantDieChance = 1;   // 1/200 chance to wither per step
  const uint8_t plantGrowBase = 40;   // 1/plantGrowBase chance to sprout if neighbors allow

  auto wrap = [](int v) { return (v + 8) & 7; };

  if (!initialized) {
    // Seed plants with moderate density and a few animals
    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        plant[y][x] = (rand() % 100 < 35) ? 1 : 0; // ~35%
        animal[y][x] = 0;
        energy[y][x] = 0;
      }
    }
    // Sprinkle animals (more herbivores than omnivores than carnivores)
    for (int i = 0; i < 10; ++i) { // 10 attempts to place animals
      int x = rand() & 7, y = rand() & 7;
      if (animal[y][x]) { --i; continue; }
      int r = rand() % 100;
      if (r < 60) { animal[y][x] = 1; energy[y][x] = 25 + (rand() % 20); }        // herbivore
      else if (r < 85) { animal[y][x] = 2; energy[y][x] = 25 + (rand() % 20); }   // omnivore
      else { animal[y][x] = 3; energy[y][x] = 30 + (rand() % 20); }               // carnivore
    }
    ticks = 0;
    initialized = true;
  }

  // Slow down evolution to make it viewable
  frameHold++;
  if (frameHold < LIFE_FRAME_DIV) {
    // Only redraw current state without evolving
    safeClearFrame();
    const uint32_t colPlant = strip.Color(0, 160, 0);
    const uint32_t colHerb  = strip.Color(200, 200, 0);
    const uint32_t colOmni  = strip.Color(255, 120, 0);
    const uint32_t colCarn  = strip.Color(255, 0, 0);
    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        if (animal[y][x] == 3) setGridPixel(x, y, colCarn);
        else if (animal[y][x] == 2) setGridPixel(x, y, colOmni);
        else if (animal[y][x] == 1) setGridPixel(x, y, colHerb);
        else if (plant[y][x]) setGridPixel(x, y, colPlant);
      }
    }
    strip.show();
    return;
  }
  frameHold = 0;

  // 1) Plants grow/wither -> plantNext
  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 8; ++x) {
      uint8_t p = plant[y][x];
      // Count neighboring plants
      int pn = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!dx && !dy) continue;
          if (plant[wrap(y + dy)][wrap(x + dx)]) pn++;
        }
      }
      // Survive with slight wither chance
      if (p) {
        if ((rand() % 200) < plantDieChance) plantNext[y][x] = 0; else plantNext[y][x] = 1;
      } else {
        // Sprout if at least 1 neighbor and a bit of luck
        if (pn >= 1 && (rand() % plantGrowBase) == 0) plantNext[y][x] = 1; else plantNext[y][x] = 0;
      }
    }
  }

  // 2) Animals act -> animalNext/energyNext; may eat plants/animals and move
  // Clear next layers
  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 8; ++x) {
      animalNext[y][x] = 0;
      energyNext[y][x] = 0;
    }
  }

  auto placeIfEmpty = [&](int tx, int ty, uint8_t type, uint8_t e) {
    if (animalNext[ty][tx] == 0) { animalNext[ty][tx] = type; energyNext[ty][tx] = e; return true; }
    return false;
  };

  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 8; ++x) {
      uint8_t a = animal[y][x];
      if (!a) continue;
      int e = (int)energy[y][x];

      // Metabolism
      if (a == 1) e -= metaHerb; else if (a == 2) e -= metaOmni; else e -= metaCarn;
      if (e <= 0) continue; // dies

      // Collect neighbors
      int nxList[8], nyList[8], nCount = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!dx && !dy) continue;
          nxList[nCount] = wrap(x + dx);
          nyList[nCount] = wrap(y + dy);
          nCount++;
        }
      }
      // Shuffle neighbor order to reduce directional bias
      for (int i = 7; i > 0; --i) { int j = rand() % (i + 1); int tx = nxList[i]; nxList[i] = nxList[j]; nxList[j] = tx; int ty = nyList[i]; nyList[i] = nyList[j]; nyList[j] = ty; }

      bool moved = false;

      auto tryMoveTo = [&](int tx, int ty, bool eatPlant, bool eatAnimal, uint8_t preyMask) {
        // preyMask bits: 1=herb, 2=omni (treated as bit2), 4=carn (unused here)
        if (eatAnimal) {
          uint8_t target = animal[ty][tx];
          if (target) {
            bool acceptable = false;
            if ((preyMask & 1) && target == 1) acceptable = true;
            if ((preyMask & 2) && target == 2) acceptable = true;
            if ((preyMask & 4) && target == 3) acceptable = true;
            if (acceptable) {
              // Move and eat (target will not be copied since next starts empty)
              int newE = e;
              if (target == 1) newE += gainHerb;
              else if (target == 2) newE += gainOmni;
              else newE += 10; // generic small gain if ever allowed
              if (newE > 255) newE = 255;
              if (placeIfEmpty(tx, ty, a, (uint8_t)newE)) { moved = true; return true; }
            }
          }
        }
        if (eatPlant) {
          if (plantNext[ty][tx]) {
            int newE = e + gainPlant; if (newE > 255) newE = 255;
            if (placeIfEmpty(tx, ty, a, (uint8_t)newE)) { plantNext[ty][tx] = 0; moved = true; return true; }
          }
        }
        // Move into empty cell
        if (animalNext[ty][tx] == 0) {
          if (placeIfEmpty(tx, ty, a, (uint8_t)e)) { moved = true; return true; }
        }
        return false;
      };

      // Preference by species
      if (a == 3) {
        // Carnivore: prefer omnivore or herbivore; else move randomly
        // Try to eat omnivore/herbivore
        for (int i = 0; i < nCount && !moved; ++i) {
          tryMoveTo(nxList[i], nyList[i], false, true, 1 | 2);
        }
        // Otherwise random move
        for (int i = 0; i < nCount && !moved; ++i) {
          tryMoveTo(nxList[i], nyList[i], false, false, 0);
        }
      } else if (a == 2) {
        // Omnivore: prefer herbivore; then plants; else random
        for (int i = 0; i < nCount && !moved; ++i) {
          tryMoveTo(nxList[i], nyList[i], false, true, 1);
        }
        for (int i = 0; i < nCount && !moved; ++i) {
          tryMoveTo(nxList[i], nyList[i], true, false, 0);
        }
        for (int i = 0; i < nCount && !moved; ++i) {
          tryMoveTo(nxList[i], nyList[i], false, false, 0);
        }
      } else {
        // Herbivore: prefer plants, else random
        for (int i = 0; i < nCount && !moved; ++i) {
          tryMoveTo(nxList[i], nyList[i], true, false, 0);
        }
        for (int i = 0; i < nCount && !moved; ++i) {
          tryMoveTo(nxList[i], nyList[i], false, false, 0);
        }
      }

      // If didn't move, try to stay (if not already taken)
      if (!moved) {
        if (animalNext[y][x] == 0) {
          animalNext[y][x] = a; energyNext[y][x] = (uint8_t)e;
        } else {
          // Nowhere to go, may die due to crowding
        }
      }
    }
  }

  // 3) Reproduction on next-state grid
  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 8; ++x) {
      uint8_t a = animalNext[y][x];
      if (!a) continue;
      if (energyNext[y][x] >= reproThresh) {
        // Try to place a child in a random neighboring empty spot
        int nxList[8], nyList[8], nCount = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (!dx && !dy) continue;
            nxList[nCount] = wrap(x + dx);
            nyList[nCount] = wrap(y + dy);
            nCount++;
          }
        }
        for (int i = 7; i > 0; --i) { int j = rand() % (i + 1); int tx = nxList[i]; nxList[i] = nxList[j]; nxList[j] = tx; int ty = nyList[i]; nyList[i] = nyList[j]; nyList[j] = ty; }
        for (int i = 0; i < nCount; ++i) {
          int tx = nxList[i], ty = nyList[i];
          if (animalNext[ty][tx] == 0) {
            animalNext[ty][tx] = a;
            energyNext[ty][tx] = childInit;
            int pe = (int)energyNext[y][x] - reproCost; if (pe < 1) pe = 1; energyNext[y][x] = (uint8_t)pe;
            break;
          }
        }
      }
    }
  }

  // Commit next state
  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 8; ++x) {
      plant[y][x]  = plantNext[y][x];
      animal[y][x] = animalNext[y][x];
      energy[y][x] = energyNext[y][x];
    }
  }
  ticks++;

  // Draw current state with priority: carn > omni > herb > plant
  safeClearFrame();
  const uint32_t colPlant = strip.Color(0, 160, 0);
  const uint32_t colHerb  = strip.Color(200, 200, 0);
  const uint32_t colOmni  = strip.Color(255, 120, 0);
  const uint32_t colCarn  = strip.Color(255, 0, 0);
  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 8; ++x) {
      if (animal[y][x] == 3) setGridPixel(x, y, colCarn);
      else if (animal[y][x] == 2) setGridPixel(x, y, colOmni);
      else if (animal[y][x] == 1) setGridPixel(x, y, colHerb);
      else if (plant[y][x]) setGridPixel(x, y, colPlant);
    }
  }
  strip.show();

  // Reset if ecosystem collapses or after many ticks
  if (ticks > 1000) { initialized = false; }
  else {
    bool anyPlant = false, anyAnimal = false;
    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        if (plant[y][x]) anyPlant = true;
        if (animal[y][x]) anyAnimal = true;
      }
    }
    if (!anyPlant || !anyAnimal) initialized = false;
  }
}

void drawPlasma() {
  safeClearFrame();
  static float tphase = 0.0f; // use float for smooth fractional stepping

  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      // Compute multi-wave plasma value in float for better scaling
  float xf = (float)x;
  float yf = (float)y;
  float t  = tphase;

      float v = 0.0f;
      v += sinf((xf + t) * 0.35f);
      v += sinf((yf + t * 0.65f) * 0.45f);
      v += sinf((xf + yf + t * 0.25f) * 0.30f);
      float dx = xf - 3.5f, dy = yf - 3.5f;
      v += sinf(sqrtf(dx * dx + dy * dy) * 0.90f + t * 0.15f);

      // v is in [-4, 4]. Normalize to [0, 255]
      int level = (int)((v + 4.0f) * (255.0f / 8.0f));
      if (level < 0) level = 0; else if (level > 255) level = 255;

      // Apply contrast expansion around midpoint 128
      int centered = level - 128;
      centered = (centered * PLASMA_CONTRAST) / 128; // >128 increases contrast
      level = centered + 128;
      if (level < 0) level = 0; else if (level > 255) level = 255;

      // Map to orange/fire palette
      uint8_t r, g, b;
      if (level < 85) {
        // Deep red to orange
        r = (uint8_t)(40 + level * 2);
        g = (uint8_t)(level / 2);
        b = 0;
      } else if (level < 170) {
        // Orange to yellow
        r = (uint8_t)(200 + (level - 85) / 2);
        g = (uint8_t)(40 + (level - 85));
        b = 0;
      } else {
        // Yellow to hot white-yellow
        r = 255;
        g = (uint8_t)(120 + (level - 170));
        b = (uint8_t)((level - 170) / 3);
      }

      setGridPixel(x, y, strip.Color(r, g, b));
    }
  }

  strip.show();
  tphase += ((float)PLASMA_SPEED) / (float)PLASMA_FRAME_DIV; // slowed animation
}

void drawBounce() {
  drawBouncer(anim.bounce.pos, anim.bounce.dir, 0, false, nullptr);
}

void drawHopChase() {
  safeClearFrame();
  safeSetPixel(anim.hop.pos, redLevel(255));
  strip.show();
  anim.hop.pos += hopStep;
  if (anim.hop.pos >= NUMPIXELS) anim.hop.pos -= NUMPIXELS;
}

void drawCometTail() {
  // Slow movement with frame counter
  if (++anim.cometFrame < COMET_STEP_FRAMES) {
    // Redraw without moving
    safeClearFrame();
    static const uint8_t tailBrightness[3] = {120, 60, 30};
    int tailLen = (SAFE_MAX_ON_PIXELS - 1 > 3) ? 3 : SAFE_MAX_ON_PIXELS - 1;
    safeSetPixel(anim.comet.pos, redLevel(255));
    for (int t = 1; t <= tailLen; ++t) {
      int idx = anim.comet.pos - t * anim.comet.dir;
      if (idx < 0) idx += NUMPIXELS;
      if (idx >= NUMPIXELS) idx -= NUMPIXELS;
      safeSetPixel(idx, redLevel(tailBrightness[t-1]));
    }
    strip.show();
    return;
  }
  anim.cometFrame = 0;
  static const uint8_t tailBrightness[3] = {120, 60, 30};
  int tailLen = (SAFE_MAX_ON_PIXELS - 1 > 3) ? 3 : SAFE_MAX_ON_PIXELS - 1;
  drawBouncer(anim.comet.pos, anim.comet.dir, tailLen, false, tailBrightness);
}

void drawLarson() {
  static const uint8_t tailBrightness[2] = {180, 60};
  int tailLen = (SAFE_MAX_ON_PIXELS - 1 > 2) ? 2 : SAFE_MAX_ON_PIXELS - 1;
  drawBouncer(anim.larson.pos, anim.larson.dir, tailLen, true, tailBrightness);
}

void drawTwinkle() {
  safeClearFrame();
  // Possibly spawn a new twinkle if there's room
  int active = 0; 
  for (int i = 0; i < MAX_TWINKLES; ++i) if (anim.twinkle.active[i]) active++;
  if (active < SAFE_MAX_ON_PIXELS) {
    if ((rand() % 6) == 0) { // ~1/6 chance per frame
      for (int i = 0; i < MAX_TWINKLES; ++i) {
        if (!anim.twinkle.active[i]) {
          anim.twinkle.active[i] = true;
          anim.twinkle.pos[i] = rand() % NUMPIXELS;
          anim.twinkle.level[i] = 255;
          break;
        }
      }
    }
  }
  // Fade and draw twinkles
  for (int i = 0; i < MAX_TWINKLES; ++i) {
    if (!anim.twinkle.active[i]) continue;
    uint8_t lvl = anim.twinkle.level[i];
    safeSetPixel(anim.twinkle.pos[i], redLevel(lvl));
    if (lvl <= twFade) { 
      anim.twinkle.active[i] = false; 
      anim.twinkle.level[i] = 0; 
    } else { 
      anim.twinkle.level[i] = (uint8_t)(lvl - twFade); 
    }
  }
  strip.show();
}

void drawDotsChase() {
  safeClearFrame();
  int maxDots = NUMPIXELS / anim.dots.spacing; if (maxDots < 1) maxDots = 1;
  int count = SAFE_MAX_ON_PIXELS < maxDots ? SAFE_MAX_ON_PIXELS : maxDots;
  for (int i = 0; i < count; ++i) {
    int pos = (anim.dots.offset + i * anim.dots.spacing) % NUMPIXELS;
    safeSetPixel(pos, redLevel(180));
  }
  strip.show();
  anim.dots.offset = (anim.dots.offset + 1) % anim.dots.spacing;
}

void drawSparkle() {
  safeClearFrame();
  if (!anim.sparkle.on) {
    if ((rand() % 8) == 0) { // Occasionally start a sparkle
      anim.sparkle.on = true;
      anim.sparkle.pos = rand() % NUMPIXELS;
      anim.sparkle.life = 3; // very short
    }
  }
  if (anim.sparkle.on && SAFE_MAX_ON_PIXELS > 0) {
    safeSetPixel(anim.sparkle.pos, redLevel(255));
    if (anim.sparkle.life > 0) anim.sparkle.life--; 
    else anim.sparkle.on = false;
  }
  strip.show();
}

void drawPingPong() {
  safeClearFrame();
  safeSetPixel(anim.ping.pos, redLevel(255));
  if (SAFE_MAX_ON_PIXELS >= 2) {
    safeSetPixel(NUMPIXELS - 1 - anim.ping.pos, redLevel(255));
  }
  strip.show();
  anim.ping.pos += anim.ping.dir;
  if (anim.ping.pos <= 0 || anim.ping.pos >= NUMPIXELS - 1) anim.ping.dir = -anim.ping.dir;
}

void drawBreath() {
  // Triangle wave between breathMin..breathMax
  int level = anim.breath.level;
  if (anim.breath.dir > 0) {
    level += anim.breath.dir;
    if (level >= 255) { level = 255; anim.breath.dir = -anim.breath.dir; }
  } else {
    level += anim.breath.dir; // breathDir negative
    if (level <= 0) { level = 0; anim.breath.dir = -anim.breath.dir; }
  }
  anim.breath.level = (uint8_t)level;

  // Map to red level range
  uint8_t red = (uint8_t)(breathMin + (long)anim.breath.level * (breathMax - breathMin) / 255);

  safeClearFrame();
  // Breathe center pixel
  int center = NUMPIXELS / 2;
  safeSetPixel(center, redLevel(red));
  strip.show();
}

void setup() {
  Serial.begin(115200); // Initialize serial for debugging
  strip.begin(); // Initialize NeoPixel strip
  strip.setBrightness(SAFE_BRIGHTNESS); // Set brightness for USB safe mode
  strip.show(); // Initialize all pixels to 'off'
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Button to cycle patterns (to GND)
  // Seed a simple RNG for twinkle/sparkle
  srand((unsigned int)(micros() ^ 0xA5A5));
  Serial.print("Pattern ");
  Serial.print((int)currentPattern);
  Serial.print(": ");
  Serial.println(patternName(currentPattern));
  Serial.println("Setup complete, starting loop");
}

void loop() {
  readButtonAndMaybeAdvance();

  switch (currentPattern) {
    case PATTERN_GRID_TEST:
      drawGridTest();
      break;
    case PATTERN_FLOWER:
      drawFlower();
      break;
    case PATTERN_WALKING:
      drawWalking();
      break;
    case PATTERN_BLINKING_EYE:
      drawBlinkingEye();
      break;
    case PATTERN_ROCKET:
      drawRocket();
      break;
    case PATTERN_GAME_OF_LIFE:
      drawGameOfLife();
      break;
     case PATTERN_PLASMA:
       drawPlasma();
       break;
    case PATTERN_BOUNCE:
      drawBounce();
      break;
    case PATTERN_HOP_CHASE:
      drawHopChase();
      break;
    case PATTERN_COMET_TAIL:
      drawCometTail();
      break;
    case PATTERN_LARSON:
      drawLarson();
      break;
    case PATTERN_TWINKLE:
      drawTwinkle();
      break;
    case PATTERN_DOTS_CHASE:
      drawDotsChase();
      break;
    case PATTERN_SPARKLE:
      drawSparkle();
      break;
    case PATTERN_PINGPONG:
      drawPingPong();
      break;
    case PATTERN_BREATH:
      drawBreath();
      break;
    default:
      drawBounce();
      break;
  }

  delay(BOUNCE_DELAY_MS); // Frame delay
}

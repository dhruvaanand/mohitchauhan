#include <Arduino.h>
#include <FastLED.h>

#define LED_PIN     5
#define COLOR_ORDER GRB
#define CHIPSET     WS2812B
#define kMatrixWidth   19
#define kMatrixHeight  19
#define NUM_LEDS (kMatrixWidth * kMatrixHeight)

#define IS_SIMULATOR 1 // Set to 1 for Wokwi, 0 for hardware

#define BRIGHTNESS 130
#define BPM 142

// Explosion particles
struct Particle {
  float x, y;
  float vx, vy;
  CRGB color;
  uint8_t life;
};
#define MAX_PARTICLES 45
Particle particles[MAX_PARTICLES];

CRGB leds[NUM_LEDS];

// ==========================================================
// MATRIX MAPPING (Serpentine)
// ==========================================================
uint16_t XY(uint8_t x, uint8_t y) {
  if (x >= kMatrixWidth || y >= kMatrixHeight) return NUM_LEDS;
  if (IS_SIMULATOR) {
    return (y * kMatrixWidth) + x;
  } else {
    if (y % 2 == 0) return (y * kMatrixWidth) + x;
    return (y * kMatrixWidth) + (18  - x);
  }
}

// ==========================================================
// UTILS & MATH
// ==========================================================
struct Point { int8_t x, y; };

void drawLine(Point a, Point b, CRGB c) {
  int dx = abs(b.x - a.x), dy = -abs(b.y - a.y);
  int sx = a.x < b.x ? 1 : -1;
  int sy = a.y < b.y ? 1 : -1;
  int err = dx + dy;
  while (true) {
    if (a.x >= 0 && a.x < kMatrixWidth && a.y >= 0 && a.y < kMatrixHeight)
      leds[XY(a.x, a.y)] = c;
    if (a.x == b.x && a.y == b.y) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; a.x += sx; }
    if (e2 <= dx) { err += dx; a.y += sy; }
  }
}

// ==========================================================
// SCENE 1: BUILDUP (0s -> 7s) - CINEMATIC LIQUID FLOW
// ==========================================================
void sceneBuildup(uint32_t ms) {
  // Cinematic Palette: Deep Blue, Neon Cyan, Soft Purple
  CRGBPalette16 wavePalette = CRGBPalette16(
    CRGB::DarkBlue,  CRGB::Blue,      CRGB::Cyan,      CRGB::DeepPink,
    CRGB::Purple,    CRGB::DarkBlue,  CRGB::Navy,      CRGB::Cyan,
    CRGB::Blue,      CRGB::Purple,    CRGB::DarkBlue,  CRGB::Cyan,
    CRGB::Navy,      CRGB::Purple,    CRGB::Blue,      CRGB::DarkBlue
  );

  // 1. Cinematic Liquid Background
  for (uint8_t y = 0; y < kMatrixHeight; y++) {
    for (uint8_t x = 0; x < kMatrixWidth; x++) {
      // Create multi-layered "ocean" noise
      uint8_t noise1 = inoise8(x * 40, y * 40 + (ms / 10), ms / 15);
      uint8_t noise2 = inoise8(x * 20 - (ms / 12), y * 30, ms / 18);
      
      uint8_t combinedNoise = qadd8(noise1 / 2, noise2 / 2);
      
      // Keep background mostly dark/black with negative space
      if (combinedNoise > 140) {
        uint8_t br = qsub8(combinedNoise, 130); // Soft glowing edges
        leds[XY(x, y)] = ColorFromPalette(wavePalette, combinedNoise + (ms / 50), br);
      }
    }
  }

  // 2. Center Floating Horizontal Visualizer
  uint8_t centerY = 9;
  uint8_t bpmWidth = beatsin8(BPM, 4, 16); // Pulse length of visualizer
  
  for (int x = 9 - (bpmWidth / 2); x <= 9 + (bpmWidth / 2); x++) {
    if (x >= 0 && x < kMatrixWidth) {
      // Waveform height modulation
      uint8_t h = inoise8(x * 100, ms / 2) / 40; // Small horizontal wave peaks
      if (h == 0) h = 1;
      
      for (int dy = -h; dy <= h; dy++) {
         int y = centerY + dy;
         if (y >= 0 && y < kMatrixHeight) {
           // Bright Cyan highlight for the visualizer
           leds[XY(x, y)] = CRGB::Cyan;
           // Soft blue glow around the white/cyan core
           if (abs(dy) > 0) leds[XY(x, y)].fadeToBlackBy(150);
         }
      }
    }
  }
}

// ==========================================================
// SCENE 2: TUNNEL (7s -> 13s) - LIQUID PLASMA VORTEX
// ==========================================================
void sceneTunnel(uint32_t ms) {
  uint32_t t = ms - 7000;
  
  // High-end EDM Palette: Electric Red, Plasma Pink, Fiery Orange
  CRGBPalette16 tunnelPalette = CRGBPalette16(
    CRGB::DarkRed,   CRGB::Red,       CRGB::DeepPink,  CRGB::HotPink,
    CRGB::OrangeRed, CRGB::DarkRed,   CRGB::Magenta,   CRGB::DarkRed,
    CRGB::Red,       CRGB::OrangeRed, CRGB::Orange,    CRGB::Yellow,
    CRGB::HotPink,   CRGB::Magenta,   CRGB::DarkRed,   CRGB::Red
  );

  for(uint8_t y=0; y<kMatrixHeight; y++) {
    for(uint8_t x=0; x<kMatrixWidth; x++) {
      // Center-relative coordinates
      float u = (x - 9.0) / 9.0;
      float v = (y - 9.0) / 9.0;
      
      float dist = sqrt(u*u + v*v);
      float angle = atan2(v, u);
      
      // Layered Plasma Interference
      // Layer 1: The Spiral
      float s1 = sin(dist * 5.0 - angle * 2.0 - t * 0.005);
      // Layer 2: Twist Interference
      float s2 = sin(angle * 3.0 + t * 0.003 + sin(dist * 4.0));
      // Layer 3: Liquid Flow
      float s3 = cos(dist * 3.0 - t * 0.004);
      
      // Combine for liquid plasma look
      float combined = (s1 + s2 + s3) / 3.0;
      uint8_t index = (combined + 1.0) * 127;
      
      // Exponential brightness for "glowing center" depth
      uint8_t br = 0;
      if (dist < 1.2) {
        br = qadd8(20, (1.0 - dist) * 200); // Inner glow
        br = qadd8(br, (combined > 0.4 ? 200 : 0)); // Highlight peaks
      }

      index += (1.0 - dist) * 100; // Shift hue towards yellow/orange at center
      
      leds[XY(x, y)] = ColorFromPalette(tunnelPalette, index, br);
      
      // Add "glossy" highlights
      if (combined > 0.8) {
        leds[XY(x, y)] += CRGB(40, 20, 20); // Soft additive white/red bloom
      }
    }
  }
}

// ==========================================================
// SCENE 3: CHORUS & BREAKDANCER (13.4s -> 20s)
// ==========================================================

struct Pose {
  Point head, neck, belly, lArm, rArm, lLeg, rLeg;
};

// Keyframes for the dance (P0 to P9)
const Pose poses[] = {
  {{9, 4}, {9, 6}, {9, 11}, {5, 8}, {13, 8}, {6, 17}, {12, 17}},   // 0: Neutral
  {{10, 3}, {9, 6}, {9, 11}, {4, 5}, {14, 5}, {4, 15}, {13, 16}}, // 1: Reach up
  {{9, 9}, {9, 11}, {9, 14}, {3, 12}, {15, 12}, {2, 18}, {16, 18}}, // 2: Squat
  {{6, 13}, {7, 11}, {9, 9}, {5, 17}, {12, 5}, {13, 14}, {10, 15}},// 3: Freeze mid-air
  {{11, 4}, {10, 6}, {9, 11}, {7, 4}, {15, 12}, {1, 10}, {9, 17}}, // 4: Kick
  {{9, 15}, {9, 13}, {9, 10}, {4, 11}, {14, 11}, {5, 4}, {13, 4}},  // 5: Handstand-ish
  {{13, 9}, {11, 9}, {9, 9}, {9, 14}, {9, 4}, {4, 5}, {4, 13}},    // 6: Windmill mid
  {{9, 2}, {9, 4}, {9, 8}, {3, 2}, {15, 2}, {5, 13}, {13, 13}},    // 7: Jump
  {{5, 7}, {7, 8}, {10, 10}, {6, 12}, {12, 6}, {8, 17}, {15, 14}}, // 8: Slide
  {{9, 16}, {9, 14}, {9, 9}, {5, 18}, {13, 18}, {6, 3}, {12, 3}},  // 9: Full Handstand
  {{10, 7}, {9, 9}, {9, 14}, {1, 3}, {10, 7}, {6, 18}, {12, 18}}   // 10: DAB
};

Point lerpPoint(Point a, Point b, uint8_t frac) {
  return { 
    (int8_t)lerp8by8(a.x, b.x, frac), 
    (int8_t)lerp8by8(a.y, b.y, frac) 
  };
}

Point rotatePoint(Point p, float angle, Point center) {
  float s = sin(angle);
  float c = cos(angle);
  float x = p.x - center.x;
  float y = p.y - center.y;
  return {
    (int8_t)(x * c - y * s + center.x),
    (int8_t)(x * s + y * c + center.y)
  };
}

void drawDancer(uint32_t ms, CRGB c) {
  uint32_t sceneMs = ms - 13400;
  
  // Final dab timing
  if (ms >= 19500) {
    Pose p = poses[10];
    for(int dx=-1; dx<=1; dx++) for(int dy=-1; dy<=1; dy++) if(abs(dx)+abs(dy)<2) leds[XY(p.head.x+dx, p.head.y+dy)] = c;
    drawLine(p.neck, p.belly, c); drawLine(p.neck, p.lArm, c); drawLine(p.neck, p.rArm, c); drawLine(p.belly, p.lLeg, c); drawLine(p.belly, p.rLeg, c);
    drawLine(p.neck, (Point){1, 3}, c); // Extra DAB arm
    return;
  }

  // Calculate interpolation
  uint16_t totalSteps = 10;
  uint16_t stepDuration = 450; 
  uint16_t currentStep = (sceneMs / stepDuration) % totalSteps;
  uint16_t nextStep = (currentStep + 1) % totalSteps;
  uint8_t frac = map(sceneMs % stepDuration, 0, stepDuration, 0, 255);

  Pose p1 = poses[currentStep];
  Pose p2 = poses[nextStep];

  Point head = lerpPoint(p1.head, p2.head, frac);
  Point neck = lerpPoint(p1.neck, p2.neck, frac);
  Point belly = lerpPoint(p1.belly, p2.belly, frac);
  Point lArm = lerpPoint(p1.lArm, p2.lArm, frac);
  Point rArm = lerpPoint(p1.rArm, p2.rArm, frac);
  Point lLeg = lerpPoint(p1.lLeg, p2.lLeg, frac);
  Point rLeg = lerpPoint(p1.rLeg, p2.rLeg, frac);

  // Add rhythmic bobbing and dynamic spinning
  int8_t bob = sin8(ms / 2) / 64 - 2;
  head.y += bob; neck.y += bob; belly.y += bob;
  lArm.y += bob; rArm.y += bob; lLeg.y += bob; rLeg.y += bob;

  // SPIN logic: If we are in certain steps (like headspin/windmill poses)
  if (currentStep == 5 || currentStep == 6 || currentStep == 9) {
    float angle = (ms % 600) * (2.0 * PI / 600.0);
    Point center = belly;
    head = rotatePoint(head, angle, center);
    neck = rotatePoint(neck, angle, center);
    lArm = rotatePoint(lArm, angle, center);
    rArm = rotatePoint(rArm, angle, center);
    lLeg = rotatePoint(lLeg, angle, center);
    rLeg = rotatePoint(rLeg, angle, center);
  }

  // Render
  for(int dx=-1; dx<=1; dx++) for(int dy=-1; dy<=1; dy++) if(abs(dx)+abs(dy)<2) leds[XY(head.x+dx, head.y+dy)] = c;
  drawLine(neck, belly, c);
  drawLine(neck, lArm, c); 
  drawLine(neck, rArm, c);
  drawLine(belly, lLeg, c);
  drawLine(belly, rLeg, c);
}

void sceneChorus(uint32_t ms) {
  // Fluid Cinematic Rainbow Background
  uint8_t hueBase = ms / 8;
  for(uint8_t y=0; y<kMatrixHeight; y++) {
    for(uint8_t x=0; x<kMatrixWidth; x++) {
      uint8_t noise = inoise8(x*30, y*30 + (ms/4), ms/10);
      leds[XY(x,y)] = CHSV(hueBase + noise, 180, 160);
    }
  }
  
  drawDancer(ms, CRGB::Black);
}

void setup() {
  delay(1000);
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
}

void loop() {
  uint32_t ms = millis() % 20000;
  FastLED.clear();
  if (ms < 7000) sceneBuildup(ms);
  else if (ms < 13000) sceneTunnel(ms);
  else if (ms < 13400) FastLED.clear();
  else sceneChorus(ms);
  FastLED.show();
}
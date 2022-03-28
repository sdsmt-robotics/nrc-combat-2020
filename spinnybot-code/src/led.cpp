#include "led.h"

LedStrip::LedStrip(int pin) : strip(NUM_LEDS, pin, NEO_GRB + NEO_KHZ800) {}

void LedStrip::init() {
  strip.begin();
  strip.setBrightness(50);
}

void LedStrip::show() { strip.show(); }

void LedStrip::fillColor(uint32_t color) {
  if (color != curColor) {
    for (int i = 0; i < NUM_LEDS; i++) {
      data[i] = color;
      strip.setPixelColor(i, data[i]);
    }
    curColor = color;
  }
}

void LedStrip::setBrightness(int b) { strip.setBrightness(b); }

uint32_t LedStrip::color(uint8_t r, uint8_t g, uint8_t b) {
  return strip.Color(r, g, b);
}
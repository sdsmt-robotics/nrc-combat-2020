#include "led.h"

LedStrip::LedStrip() {}

void LedStrip::fillColor(CRGB color) {
  if (color != curColor) {
    for (int i = 0; i < NUM_LEDS; i++) {
      data[i] = color;
    }
    curColor = color;
  }
}

CRGB *LedStrip::getPixelData() { return data; }
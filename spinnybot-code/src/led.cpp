#include "led.h"

// Define some colors
const uint32_t LedStrip::YELLOW = Adafruit_NeoPixel::Color(255, 255, 0);
const uint32_t LedStrip::RED = Adafruit_NeoPixel::Color(255, 0, 0);
const uint32_t LedStrip::GREEN = Adafruit_NeoPixel::Color(0, 255, 0);
const uint32_t LedStrip::BLUE = Adafruit_NeoPixel::Color(0, 0, 255);
const uint32_t LedStrip::PURPLE = Adafruit_NeoPixel::Color(255, 0, 255);
const uint32_t LedStrip::WHITE = Adafruit_NeoPixel::Color(255, 255, 255);

LedStrip::LedStrip(int pin) : strip(NUM_LEDS, pin, NEO_GRB + NEO_KHZ800) {}

void LedStrip::init() {
  strip.begin();
  strip.setBrightness(128);
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

void LedStrip::setBrightness(uint8_t b) { strip.setBrightness(b); }

uint32_t LedStrip::makeColor(uint8_t r, uint8_t g, uint8_t b) {
  return strip.Color(r, g, b);
}
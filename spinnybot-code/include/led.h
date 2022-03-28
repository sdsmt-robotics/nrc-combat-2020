#ifndef LED_H
#define LED_H

// #define FASTLED_RMT_BUILTIN_DRIVER 1
// #include <FastLED.h>

#include <Adafruit_NeoPixel.h>

#define NUM_LEDS 8

class LedStrip {
public:
  LedStrip(int pin);
  void init();
  void show();
  void fillColor(uint32_t color);
  void setBrightness(int b);

  uint32_t color(uint8_t r, uint8_t g, uint8_t b);

private:
  uint32_t data[NUM_LEDS];
  uint32_t curColor;
  Adafruit_NeoPixel strip;
};

#endif
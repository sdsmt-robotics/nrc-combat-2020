#ifndef LED_H
#define LED_H

#include <FastLED.h>

#define NUM_LEDS 8

class LedStrip {
public:
  LedStrip();
  void fillColor(CRGB color);
  CRGB *getPixelData();

private:
  CRGB data[NUM_LEDS];
  CRGB curColor;
};

#endif
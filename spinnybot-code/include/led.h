#ifndef LED_H
#define LED_H

#include <Adafruit_NeoPixel.h>

#define NUM_LEDS 8

class LedStrip {
public:
  LedStrip(int pin);
  void init();
  void show();
  void fillColor(uint32_t color);
  void setBrightness(uint8_t b);

  uint32_t makeColor(uint8_t r, uint8_t g, uint8_t b);

  static const uint32_t YELLOW;
  static const uint32_t RED;
  static const uint32_t GREEN;
  static const uint32_t BLUE;
  static const uint32_t PURPLE;
  static const uint32_t WHITE;

private:
  uint32_t data[NUM_LEDS];
  uint32_t curColor;
  Adafruit_NeoPixel strip;
};

#endif
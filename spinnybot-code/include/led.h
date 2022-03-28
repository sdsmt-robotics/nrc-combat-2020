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

  const uint32_t YELLOW = Adafruit_NeoPixel::Color(255, 255, 0);
  const uint32_t RED = Adafruit_NeoPixel::Color(255, 0, 0);
  const uint32_t GREEN = Adafruit_NeoPixel::Color(0, 255, 0);
  const uint32_t BLUE = Adafruit_NeoPixel::Color(0, 0, 255);
  const uint32_t PURPLE = Adafruit_NeoPixel::Color(255, 0, 255);
  const uint32_t WHITE = Adafruit_NeoPixel::Color(255, 255, 255);

private:
  uint32_t data[NUM_LEDS];
  uint32_t curColor;
  Adafruit_NeoPixel strip;
};

#endif
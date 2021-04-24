/**
 * @file LedStrip.cpp
 * 
 * Class for a filter. Used to filter some input based on the history of that value.
 * 
 * 
 * int filter(int newVal) - add a raw value and return the filtered value.
 * int getFilteredVal() - get the filtered value
 * void reset(int fillVal = 0) - reset the filter buffer
 * 
 */

#include "LedStrip.h"

LedStrip::LedStrip() {
}

void LedStrip::fillColor(CRGB color) {
  if (color != curColor) {
    for (int i = 0; i < NUM_LEDS; i++) {
      data[i] = color;
    }
    curColor = color;
  }
}

CRGB* LedStrip::getPixelData() {
  return data;
}

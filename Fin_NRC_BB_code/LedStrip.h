/* 
 * Header for the filter class.
 */

#ifndef LED_STRIP_H
#define LED_STRIP_H

#include "Arduino.h"

#include <FastLED.h>

class LedStrip {
public:
    LedStrip();

    void fillColor(CRGB color);
    CRGB* getPixelData();

private:
    const int NUM_LEDS = 8;
    CRGB data[8];
    CRGB curColor;
};

#endif

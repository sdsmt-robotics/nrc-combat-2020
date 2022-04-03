/**
 * Code to test the LED strips. Will cycle through the strips setting colors.
 * To run: pio run -t upload -c examples.ini -e led_strips
 */
#include <Arduino.h>

#include "led.h"
#include "pins.h"

LedStrip strip1(LED_STRIP_1);
LedStrip strip2(LED_STRIP_2);
LedStrip strip3(LED_STRIP_3);
LedStrip* strips[] = {&strip1, &strip2, &strip3};
const unsigned NUM_STRIPS = sizeof(strips) / sizeof(LedStrip*); 

/**
   @brief Updates all LED strips to set colors.
*/
void showLEDStrips() {
  strip1.show();
  strip2.show();
  strip3.show();
}

void setup() {
    Serial.begin(115200);

    // init strips
    Serial.println("Initializing strips...");
    strip1.init();
    strip2.init();
    strip3.init();
    Serial.println("Done!");
}

void loop() {
    static unsigned curIdx = 0;

    // Set strips to colors
    for (int i = 0; i < NUM_STRIPS; i++) {
        strips[curIdx % NUM_STRIPS]->fillColor(LedStrip::RED);
        strips[(curIdx+1) % NUM_STRIPS]->fillColor(LedStrip::GREEN);
        strips[(curIdx+2) % NUM_STRIPS]->fillColor(LedStrip::BLUE);
    }
    showLEDStrips();
    Serial.println(curIdx);

    curIdx = (curIdx + 1) % NUM_STRIPS;
    delay(1000);
}
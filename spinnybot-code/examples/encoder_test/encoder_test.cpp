/**
 * Code to test the motor encoder reading. Will read velocity from all encoders.
 * To run: pio run -t upload -c examples.ini -e encoder_test
 */
#include <Arduino.h>

#include "pins.h"
#include "encoder.h"

#define WHEEL_RADIUS 29 // Wheel radius in mm
#define BOT_RADIUS 130  // Robot chassis radius in mm

// Create the encoder objects
Encoder e1(ENC_1, 360);
Encoder e2(ENC_2, 360);
Encoder e3(ENC_3, 360);
Encoder* encoders[] = {&e1, &e2, &e3};
const unsigned NUM_ENCODERS = sizeof(encoders) / sizeof(Encoder*);

/**
 * Estimate the bot's angular velocity.
 * 
 * @param encoders - Array of encoders on the bot.
 * @param numEncoders - Number of encoders
 * @return The estimated bot velocity
 */
float estimateBotRpm(Encoder* encoders[], unsigned numEncoders) {
    float sum = 0;
    for (int i = 0; i < numEncoders; i++) {
        sum += encoders[i]->getSpeed();
    }
    return sum * WHEEL_RADIUS / float(numEncoders * BOT_RADIUS);
}

void setup() {
    Serial.begin(115200);

    // init encoders
    e1.init();
    e2.init();
    e3.init();
}

void loop() {
    static unsigned long lastPrint = 0;
    static unsigned long lastUpdate = 0;

    // Update encoder velocity estimates
    if (millis() - lastUpdate > 2) {
        for (int i = 0; i < NUM_ENCODERS; i++) {
            encoders[i]->update();
        }
        lastUpdate = millis();
    }

    // Print current values
    if (millis() - lastPrint > 250) {
        for (int i = 0; i < NUM_ENCODERS; i++) {
            Serial.printf("E%d:[p:%0.2f, v:%d], ", i+1, encoders[i]->getPos(), encoders[i]->getSpeed());
        }
        Serial.printf("bot:%f\n", estimateBotRpm(encoders, NUM_ENCODERS));
        lastPrint = millis();
    }
}
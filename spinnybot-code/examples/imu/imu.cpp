/**
 * Code to test the IMU.
 * To run: pio run -t upload -c examples.ini -e imu
 */
#include "imu.h"

#include <Arduino.h>

IMU imu;

const long LOOP_INTERVAL = 1000;  // microsec
const long PRINT_INTERVAL = 500;  // millisec

void setup() {
    Serial.begin(115200);

    // init IMU
    Serial.println("Initializing IMU...");
    if (!imu.init()) {
        Serial.println("IMU fail init.");
        while (true) {delay(1);}
    } 
    Serial.println("Done!");
}

void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastPrint = 0;

    // update IMU at fixed interval
    if (micros() - lastUpdate > LOOP_INTERVAL) {
        imu.update();
        lastUpdate = millis();
    }

    // Print at fixed interval
    if (millis() - lastPrint > PRINT_INTERVAL) {
        Serial.print(imu.getVelocity() * RAD_TO_DEG, 1);
        Serial.print("\t");
        Serial.println(imu.getAngle() * RAD_TO_DEG, 1);
        lastPrint = millis();
    }
}
/**
 * Code to demo OTA updating. Sets up an AP and allows OTA updates.
 * To run: pio run -t upload -c examples.ini -e ota_updates
 * 
 * To do the OTA update:
 *  - Connect to the hosted wifi network
 *  - Run pio run -t upload -c examples.ini -e ota_updates-ota'
 */

#include <Arduino.h>
#include "ota.h"

#define AP_SSID "SPINNY"

void setup() {
    Serial.begin(115200);
    Serial.println("Booting.");

    // Initialize the access point and OTA udpater
    Serial.print("Initializing AP...");
    bool success = initAp(AP_SSID);
    Serial.println(success ? "done!" : "fail.");
    if (success) {
        Serial.print("Initializing OTA...");
        initOta();
        Serial.println("Ready!");
    }
}

void loop() {
}
/**
 * Demo class for drive code.
 * to run: pio run -t upload -c examples.ini -e drive_test
 */
#include <Arduino.h>

#include "controller.h"
#include "driveFuncs.h"
#include "pins.h"

// Create the communications object. Use Serial for the communications.
Controller controller(CONTROLLER_SERIAL);

//=====DEMOS=============================================

/**
 * Spam all of the controller values unto the screen.
 */
void printEverything() {
    static unsigned long lastPrint = millis();

    if (millis() - lastPrint > 100) {
        // Format:
        // dpad:[lrud],but:[lrud],bump:[lr],trig:[l,r],joyL:[l,r],joyR:[l,r]

        // Dpad
        Serial.print("dpad:[");
        Serial.print(controller.dpad(LEFT));
        Serial.print(controller.dpad(RIGHT));
        Serial.print(controller.dpad(UP));
        Serial.print(controller.dpad(DOWN));

        // buttons
        Serial.print("],but:[");
        Serial.print(controller.button(LEFT));
        Serial.print(controller.button(RIGHT));
        Serial.print(controller.button(UP));
        Serial.print(controller.button(DOWN));

        // bumpers
        Serial.print("],bump:[");
        Serial.print(controller.bumper(LEFT));
        Serial.print(controller.bumper(RIGHT));

        // joysticks
        Serial.print("],joy:[");
        Serial.print(controller.joyButton(LEFT));
        Serial.print(controller.joyButton(RIGHT));

        // triggers
        Serial.print("],trig:[");
        Serial.print(controller.trigger(LEFT));
        Serial.print(",");
        Serial.print(controller.trigger(RIGHT));

        // left joystick
        Serial.print("],joyL:[");
        Serial.print(controller.joystick(LEFT, X));
        Serial.print(",");
        Serial.print(controller.joystick(LEFT, Y));

        // right joystick
        Serial.print("],joyR:[");
        Serial.print(controller.joystick(RIGHT, X));
        Serial.print(",");
        Serial.print(controller.joystick(RIGHT, Y));
        Serial.println("]");

        lastPrint = millis();
    }
}

//=====SETUP=============================================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10000);

    // initialize the receiver
    controller.init();
    Serial.println("Waiting for connection...");
    while (!controller.connected()) {
        delay(10);
    }
    Serial.println("Connected!");

    // set a deadzone for the joysticks
    controller.setJoyDeadzone(0.08);
}

//=====MAIN LOOP=============================================
void loop() {
    static unsigned long lastPrint = 0;
    static float botAngle = 0;
    static int powers[3] = {0};
    static bool disconnected = false;
    static float spinDir = 0.0001;

    // Uncomment a demo mode to run it.
    if (controller.connected()) {
        // Print all values to the serial monitor

        // Check for any incoming commands
        if (Serial.available()) {
            switch (Serial.read()) {
                case 'a': // Set angle
                    botAngle = -(Serial.parseFloat() * DEG_TO_RAD);  // Invert since gyro vals are inverted
                    delay(500);
                    while (Serial.available()) Serial.read();
                    break;
                case 'i':  // Invert
                    spinDir = -spinDir;
                    break;

                default:
                    break;
            }
        }

        // Print status
        if (millis() - lastPrint > 500) {
            float x = -controller.joystick(RIGHT, X);  // Invert x since it is backwards for some reason...
            float y = controller.joystick(RIGHT, Y);
            calcDrivePower(powers, spinDir, x, y, botAngle);
            Serial.printf("[x:%0.2f,y:%0.2f,a:%0.2f,i:%d]->[%d, %d, %d]\n", x, y, botAngle, (spinDir<0), powers[0], powers[1], powers[2]);
            lastPrint = millis();
        }

        disconnected = false;
    } else {
        if (!disconnected) {
            Serial.println("Disconnected! Waiting for reconnect...");
            disconnected = true;
        }
    }
}

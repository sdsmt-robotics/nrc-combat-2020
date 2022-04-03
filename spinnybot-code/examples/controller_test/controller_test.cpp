/**
 * Demo class for the controller receive code.
 *
 * Uncomment a function in loop() to test out some functionality.
 *
 * This uses an xbee connected to a serial port.
 */
#include <Arduino.h>
#include "pins.h"
#include "Controller.h"

bool disconnected = false;

// Create the communications object. Use Serial for the communications.
Controller controller(CONTROLLER_SERIAL);


//=====DEMOS=============================================
/**
 * Display when one of the buttons get pressed (clicked).
 */
void printButtonChanges() {
    // right side buttons
    if (controller.buttonClick(UP)) {
        Serial.println("btn:UP");
    }
    if (controller.buttonClick(DOWN)) {
        Serial.println("btn:DOWN");
    }
    if (controller.buttonClick(LEFT)) {
        Serial.println("btn:LEFT");
    }
    if (controller.buttonClick(RIGHT)) {
        Serial.println("btn:RIGHT");
    }

    // Dpad
    if (controller.dpadClick(UP)) {
        Serial.println("dpad:UP");
    }
    if (controller.dpadClick(DOWN)) {
        Serial.println("dpad:DOWN");
    }
    if (controller.dpadClick(LEFT)) {
        Serial.println("dpad:LEFT");
    }
    if (controller.dpadClick(RIGHT)) {
        Serial.println("dpad:RIGHT");
    }

    // bumpers
    if (controller.bumperClick(LEFT)) {
        Serial.println("bumper:LEFT");
    }
    if (controller.bumperClick(RIGHT)) {
        Serial.println("bumper:RIGHT");
    }

    // joysticks
    if (controller.joyButtonClick(LEFT)) {
        Serial.println("joy:LEFT");
    }
    if (controller.joyButtonClick(RIGHT)) {
        Serial.println("joy:RIGHT");
    }
}

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

        if (abs(controller.joystick(RIGHT, X)) < 0.001 || abs(controller.joystick(RIGHT, Y)) < 0.001 || abs(controller.joystick(LEFT, X)) < 0.001 || abs(controller.joystick(LEFT, Y)) < 0.001) {
            // Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        }

        lastPrint = millis();
    }
}

//=====SETUP=============================================
void setup() {
    Serial.begin(115200);

    // initialize the receiver
    controller.init();
    Serial.println("Waiting for connection...");
    while (!controller.connected()) {
        delay(10);
    }
    Serial.println("Connected...");

    // set a deadzone for the joysticks
    controller.setJoyDeadzone(0.08);
}

//=====MAIN LOOP=============================================
void loop() {
    static unsigned long lastPrint = 0;

    // Uncomment a demo mode to run it.
    if (controller.connected()) {
        // Print all values to the serial monitor
        if (millis() - lastPrint > 250) {
            printEverything();
            lastPrint = millis();
        }

        // only print button changes (clicks)
        // printButtonChanges();

        disconnected = false;
    } else {
        if (!disconnected) {
            Serial.println("Disconnected! Waiting for reconnect...");
            disconnected = true;
        }
    }
}

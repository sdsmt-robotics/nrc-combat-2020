/**
 * Example code to test motor control. 
 *  - ' ' = Stop all motors
 *  - '#' = (1,2,3) Run a given motor
 *  - 'r' = Read off current values being sent to motors.
 *  - 's' = Set all motors to run
 * 
 * To run: pio run -t upload -c examples.ini -e motor_test
 */

#include <Arduino.h>

#include "motor.h"
#include "pins.h"

#define DEBUG_PRINTLN(str) Serial.println(str)  // For testing
//#define DEBUG_PRINTLN   // For release

// ---- MOTOR STUFF ----

Motor m1(MOTOR_1, MOTOR_1_RMT_CHANNEL);
Motor m2(MOTOR_2, MOTOR_2_RMT_CHANNEL);
Motor m3(MOTOR_3, MOTOR_3_RMT_CHANNEL);
Motor* motors[] = {&m1, &m2, &m3};
const unsigned NUM_MOTORS = sizeof(motors) / sizeof(Motor*);

const int SPIN_MIN_POWER = 100;  // min motor power when enabled

const long MOTOR_INIT_TIME = 4000;  // wait time for motor init in miliseconds

const long MOTOR_UPDATE_INTERVAL = 1000;  // motor update loop time in microseconds
long last_motor_update = micros();

// Helper function for getting user input
void flushSerial() {
    while (Serial.available()) Serial.read();
}

// ---- MOTOR FUNCTIONS ----

/**
   @brief Initializes all motors.
*/
void initMotors() {
    m1.init();
    m2.init();
    m3.init();
}

/**
   @brief Sends a DShot command to all motors.
   @param cmd command
*/
void sendAllMotorCommand(int cmd) {
    m1.send_dshot_command(cmd);
    m2.send_dshot_command(cmd);
    m3.send_dshot_command(cmd);
}

/**
   @brief Sets a power for all motors.
   @param power power value to set. -1000 to 1000
*/
void sendAllMotorPower(int power) {
    m1.set_speed_signed(power);
    m2.set_speed_signed(power);
    m3.set_speed_signed(power);
}

/**
   @brief Arms all motors by initialy sending the 0 command and then sets the
   power to 0 for the initialization. This blocks for MOTOR_INIT_TIME
   miliseconds.
*/
void armMotors() {
    long init_motor_start = millis();
    sendAllMotorCommand(0);
    while (millis() - init_motor_start < MOTOR_INIT_TIME) {
        sendAllMotorPower(0);
    }
}


// ---- MAIN FUNCTIONS ----

void setup() {
    Serial.begin(115200);

    DEBUG_PRINTLN("Initializing motors...");
    initMotors();
    armMotors();

    DEBUG_PRINTLN("Done! Beginning main loop.");
}

void loop() {
    char readVal = 0;
    static int powers[NUM_MOTORS] = {SPIN_MIN_POWER, SPIN_MIN_POWER, SPIN_MIN_POWER};
    static unsigned lastMotorUpdate = 0;

    // Check if any serial data available
    while (Serial.available()) {
        readVal = Serial.read();
    }

    // Check for request to stop all motors
    if (readVal == ' ') {  // Stop all motors
        Serial.println("Stopping all.");
        for (int i = 0; i < NUM_MOTORS; i++) {
            powers[i] = 0;
        }
    } else if (readVal == 'r') {  // Read off all values
        for (int i = 0; i < NUM_MOTORS; i++) {
            Serial.printf("%d, ", powers[i]);
        }
        Serial.print("\n");
    } else if (readVal == 's') {  // Set all
        Serial.println("Setting all.");
        for (int i = 0; i < NUM_MOTORS; i++) {
            powers[i] = SPIN_MIN_POWER;
        }
    } else {
        // Control the selected motor (If given one)
        int mtrNum = readVal - '1';
        if (mtrNum >= 0 && mtrNum < NUM_MOTORS) {
            // Get the speed from the user
            Serial.printf("Setting motor [%d].\n", mtrNum+1);
            powers[mtrNum] = SPIN_MIN_POWER;
        }
    }

    // motor update timer
    if (micros() - last_motor_update > MOTOR_UPDATE_INTERVAL) {
        last_motor_update = micros();

        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i]->set_speed_signed(powers[i]);
        }
    }
}

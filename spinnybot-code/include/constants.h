#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

// ---- MISC ----

const float RPM_TO_RAD_SEC = TWO_PI / 60.0;
const float PI2_OVER_3 = TWO_PI / 3;

// ---- MOTOR constants ----

const unsigned NUM_MTRS = 3;
const unsigned WHEEL_RADIUS = 30; // Wheel radius in mm
const unsigned BOT_RADIUS = 128.6;  // Robot chassis radius in mm
const unsigned RAD_RATIO = WHEEL_RADIUS / BOT_RADIUS;
// Offsets of the three motors from "forward" on the bot
const float MTR_OFFSETS[] = {PI2_OVER_3, 0.0, -PI2_OVER_3};

const int SPIN_MAX_POWER = 300; // max motor power when enabled
const int SPIN_MIN_POWER = 100; // min motor power when enabled

// max movement power. this is the magnitude of max power when moving
const int TRANS_MAX_POWER = 50;

const int FULL_SEND_POWER = 500; // power to use for full send mode

const long MOTOR_INIT_TIME = 2000; // wait time for motor init in miliseconds


#endif
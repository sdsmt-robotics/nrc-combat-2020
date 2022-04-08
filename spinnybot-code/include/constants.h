#ifndef CONSTANTS_H
#define CONSTANTS_H

// ---- MOTOR constants ----
const int SPIN_MAX_POWER = 300; // max motor power when enabled
const int SPIN_MIN_POWER = 100; // min motor power when enabled

// max movement power. this is the magnitude of max power when moving
const int TRANS_MAX_POWER = 50;

const int FULL_SEND_POWER = 800; // power to use for full send mode

const long MOTOR_INIT_TIME = 2000; // wait time for motor init in miliseconds

// ---- MISC ----

const float PI2_OVER_3 = TWO_PI / 3;

#endif
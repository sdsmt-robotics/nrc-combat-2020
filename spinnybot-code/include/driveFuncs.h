/**
 * Functions used for driving the BB.
 */

#ifndef __BB_DRIVE__
#define __BB_DRIVE__

#include "encoder.h"
#include "imu.h"

float updateOrientation(IMU &imu, Encoder &e1, Encoder &e2, Encoder &e3);
void calcDrivePower(int power[3], float spin, float x, float y, float angle);


#endif
#include "driveFuncs.h"
#include "constants.h"

/**
   @brief Updates all sensors.
*/
float updateOrientation(IMU &imu, Encoder &e1, Encoder &e2, Encoder &e3) {
  // Update the estimates from all sensors
  imu.update();

  // update accelerometers
  // accel.update();

  // update encoders
  e1.update();
  e2.update();
  e3.update();

  // Track fastest motor speed around a full revolution to figure out heading
  // angle

  return imu.getAngle();
}

/**
   @brief Updates drive motor powers with control and angle inputs.
   @param power array of power values for motors.
   @param spin Spin speed modifier. -1.0 to 1.0
   @param x X direction movement modifier. -1.0 to 1.0
   @param y Y direction movement modifier. -1.0 to 1.0
   @param angle Current angle of robot in radians
*/
void calcDrivePower(int power[3], float spin, float x, float y, float angle) {
  // Offsets of the three motors from "forward" on the bot
  const float m1_offset = PI2_OVER_3;
  const float m2_offset = 0.0;
  const float m3_offset = -PI2_OVER_3;

  // Phase offset to make the bot go in the intended direction
  float phase = PI2_OVER_3; //! This is a complete guess
  // float phase = 0;

  // constrain inputs to prevent problems with math
  spin = constrain(spin, -1.0, 1.0);
  x = constrain(x, -1.0, 1.0);
  y = constrain(y, -1.0, 1.0);

  float spin_power = 0;
  float trans_power = TRANS_MAX_POWER * sqrtf((x * x) + (y * y));
  float trans_phase = atan2f(y, x) - HALF_PI;

  if (spin > 0) {
    // robot upright
    float comp_angle = trans_phase + angle + phase;

    spin_power = SPIN_MIN_POWER + ((SPIN_MAX_POWER - SPIN_MIN_POWER) * spin);

    power[0] = (spin_power + (trans_power * sinf(m1_offset - comp_angle)));
    power[1] = (spin_power + (trans_power * sinf(m2_offset - comp_angle)));
    power[2] = (spin_power + (trans_power * sinf(m3_offset - comp_angle)));

  } else if (spin < 0) {
    // robot fliped over
    float comp_angle = trans_phase - angle - phase;

    spin_power = -SPIN_MIN_POWER + ((SPIN_MAX_POWER - SPIN_MIN_POWER) * spin);

    power[0] = (spin_power + (trans_power * sinf(m1_offset + comp_angle)));
    power[1] = (spin_power + (trans_power * sinf(m2_offset + comp_angle)));
    power[2] = (spin_power + (trans_power * sinf(m3_offset + comp_angle)));
  }
}
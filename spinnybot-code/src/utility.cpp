/**
 * Some common functions used across the program.
 */
#include "utility.h"
#include <Arduino.h>

/**
 * Normalize an angle to the range [-PI, PI].
 *
 * @param angle - The angle to normalize.
 * @return the normalized angle
 */
float normalizeAngle(float angle) {
  if (angle > PI) {
    do {
      angle -= TWO_PI;
    } while (angle > PI);
  } else {
    while (angle < -PI) {
      angle += TWO_PI;
    }
  }
  return angle;
}

/**
 * Check if the given floating point value is zero.
 * 
 * @param val - The value to test.
 * @param epsilon - Value to test within.
 * @return true if close to zero, false otherwise.
 */
bool isZero(float val, float epsilon) {
    return val < epsilon && val > -epsilon;
}


/**
 * Get the difference between the two angles in the given direction.
 * 
 * @param from - First angle.
 * @param to - Second angle.
 * @param positive - Whether to go in the positive direction.
 * @return Difference (a1 - a2).
 */
float angleTo(float from, float to, bool positive) {
    float diff = to - from;

    if (positive) {
        // If < zero, increase until >= zero 
        while (diff + 0.000001 < 0) {
            diff += TWO_PI;
        }
    } else {
        // If > zero, decrease until <= zero 
        while (diff - 0.000001 > 0) {
            diff -= TWO_PI;
        }
    }

    return diff;
}
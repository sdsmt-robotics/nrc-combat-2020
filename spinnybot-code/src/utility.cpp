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
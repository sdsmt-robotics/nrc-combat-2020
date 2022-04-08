/**
 * Some common functions used across the program.
 */
#ifndef UTILITY_H
#define UTILITY_H

float normalizeAngle(float angle);
bool isZero(float val, float epsilon=0.0001);
float angleTo(float from, float to, bool positive);

#endif
// 1/7/2020
// Samuel Ryckman
//
// Class for the LY530AL and LPR530AL gyro sensor pair. 

#include "L530AL.h"

// Define the default pins
#define GYRO_X_DEFAULT A1
#define GYRO_Y_DEFAULT A2
#define GYRO_Z_DEFAULT A0

/** Default constructor, uses default anolog pins.
 */
L530AL::L530AL() : L530AL(GYRO_X_DEFAULT, GYRO_Y_DEFAULT, GYRO_Z_DEFAULT) { }


/** Constructor with non-default pins.
 * 
 * @param xPin - anolog pin for the x value
 * @param yPin - anolog pin for the y value
 * @param zPin - anolog pin for the z value
 */
L530AL::L530AL(uint8_t xPin, uint8_t yPin, uint8_t zPin)
    : xPin(xPin),
      yPin(yPin),
      zPin(zPin) { }


/** Get the angle values from the devices.
 * 
 * @param x - the x value from the sensor
 * @param y - the y value from the sensor
 * @param z - the z value from the sensor
 */
void L530AL::getVals(uint16_t &x, uint16_t &y, uint16_t &z) {
  x = getX();
  y = getY();
  z = getZ();
}


/** Get the x angle value from the device.
 * 
 * @return the x value from the sensor
 */
uint16_t L530AL::getX() {
    return analogRead(xPin);
}


/** Get the y angle value from the device.
 * 
 * @return the y value from the sensor
 */
uint16_t L530AL::getY() {
    return analogRead(yPin);
}


/** Get the z angle value from the device.
 * 
 * @return the z value from the sensor
 */
uint16_t L530AL::getZ() {
    return analogRead(zPin);
}
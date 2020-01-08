// 1/7/2020
// Samuel Ryckman
//
// Demo Arduino sketch for the gyro sensors on the Sparkfun Razor IMU.
// The Razor uses a pair of gyro sensor with the LY530AL handling the z-axis
// and the LPR530AL handling the x-axis and y-axis. 

#include "L530AL.h"

// Construct the gyro object
L530AL gyro;


void setup() {
    // initialize serial communication
    Serial.begin(38400);
}

void loop() {
  static uint16_t gyroX, gyroY, gyroZ;
  
  //read the gyro
  gyro.getVals(gyroX, gyroY, gyroZ);

  // display tab-separated accel x/y/z values
  Serial.print("accel:\t");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.println(gyroZ);

  delay(20);
}


// 1/7/2020
// Samuel Ryckman
//
// Get calibration values for the magnetometer.
// 
// Instructions: upload this sketch to the imu. Then rotate the imu around 
// in EVERY direction until the values in the serial monitor stop changing.
//
// All directions:  x: (-580, 709)  x: (-613, 692)  x: (-679, 553)  
// In xy plane:     x: (-529, 640)  y: (-718, 601)  z: (-1284, 0) 

#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5843.h"

//Create the magnatometer device
HMC5843 mag;

#define LED_PIN 13
bool blinkState = false;


//Update the values if new value is larger or smaller.
void updateMaxMin(int16_t &maxVal, int16_t &minVal, int16_t newVal) {
  if (newVal > maxVal) {
    maxVal = newVal;
  } else if (newVal < minVal) {
    minVal = newVal;
  }
}

void setup() {
    // join I2C bus
    Wire.begin();

    // initialize serial communication
    Serial.begin(38400);

    // initialize device
    //Serial.println("Initializing devices...");
    mag.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(mag.testConnection() ? "HMC5843 connection successful" : "HMC5843 connection failed");

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
  static int16_t mx, my, mz;
  static int16_t maxX = 0, maxY = 0, maxZ = 0;
  static int16_t minX = 0, minY = 0, minZ = 0;
  
  // Get the heading from the device
  mag.getHeading(&mx, &my, &mz);

  //update the values
  updateMaxMin(maxX, minX, mx);
  updateMaxMin(maxY, minY, my);
  updateMaxMin(maxZ, minZ, mz);
  
  // Display the values
  Serial.print("x: ("); Serial.print(minX); Serial.print(", "); Serial.print(maxX); Serial.print(")\t");
  Serial.print("y: ("); Serial.print(minY); Serial.print(", "); Serial.print(maxY); Serial.print(")\t");
  Serial.print("z: ("); Serial.print(minZ); Serial.print(", "); Serial.print(maxZ); Serial.println(")\t");
  
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  delay(100);
}

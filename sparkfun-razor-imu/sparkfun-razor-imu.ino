// 1/7/2020
// Samuel Ryckman
// Send values to the connected device over a serial connection.

#include "Wire.h"
#include "math.h"
#include "I2Cdev.h"
#include "HMC5843.h"

//Create the magnatometer device
HMC5843 mag;

#define REQUEST_DATA 5  //Request data (Enquiry control character)
#define LED_PIN 13
bool blinkState = false;

// Struct for holding calibration values.
// Raw values from the sensor are offset from 0 by some amount 
// and not scaled uniformily. We can fix this using the max 
// and min values from the sensor which are found during calibration.
struct CalVals {
  int16_t shift;  //amount to shift the given value
  float scale;  //scaling multiplier for the given value

  //Constructor using the max and min values
  CalVals(int16_t maxVal, int16_t minVal) {
    const int16_t range = 1000;  //range for the output values
    
    shift = -(maxVal + minVal) / 2;
    scale = range / ((maxVal - minVal) / 2.0);
  }
  
} ;

/* Get the direction in degrees (-180 to 180) from the imu.
 */
float getDirection() {
  // Define the calibration values.
  const CalVals xCal(640, -529);
  const CalVals yCal(601, -718);

  //values from the sensor
  int16_t xVal, yVal, zVal;

  //Get the values
  mag.getHeading(&xVal, &yVal, &zVal);

  //Scale and shift the values
  xVal = (xVal + xCal.shift) * xCal.scale;
  yVal = (yVal + yCal.shift) * yCal.scale;
  
  //Calculate the angle in degrees x 10
  return atan2(yVal, xVal) * 180 / PI;
  
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
  static float dir;
  static char transmitByte;

  //Wait for a value to be received
  if (Serial.available() > 0) {
    // Check if it is a request for data
    if (Serial.read() == REQUEST_DATA) {
      // Get the heading from the device
      dir = getDirection();
      
      // Send the value
      Serial.write((char*)(&dir), sizeof(float));
      //Serial.println(dir);
      
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  }
}

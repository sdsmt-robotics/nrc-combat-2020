/************************************************************************/ /**
*@file
*@brief Contains the code for the IMU.
***************************************************************************/
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_ICM20649.h>
#include <math.h>
#include "Filter.h"


class Imu {
private:
   float imuOffset = 0;  // Angle offset of the IMU
   
   // Structs for reading from the sensor
   sensors_event_t accel;
   sensors_event_t gyro;
   sensors_event_t temp;

   float accelZ = 0.0;  // Accelerometer value in the z-direction
   
   float angle = 0.0; // Current estimate of the angle we are facing

   Filter angleRateFilter = Filter(3); //set up a averaging low pass filter
   float gyroVals[3] = {0, 0, 0};  // Rolling array of gyro values
   int valsMidIdx = 0;  // Index of the middle of the rolling array
   float gyroToRad = 0.852;   // Convert gyro value to rad/s
   
   Filter accelZFilter = Filter(3); // Filter for z-axis accelerometer
   
   unsigned long lastUpdateTime = micros(); //loop start time

   // Calibration stuff
   float gyroDrift = 0.0;  // Average value from the gyro while standing still
   const int CALIBRATE_READ_WAIT = 5;       // Time in ms to wait between calibration reads
   const int CALIBRATE_NUM_READINGS = 100;  // Number of calibration reads

   bool readImu();                                         //get raw values adjusted by average static error
   float integrateGyro(float deltaT); //integrate values

   Adafruit_ICM20649 IMU;
   unsigned csPin = SS;

public:
   Imu(unsigned csPin, float imuOffset = 0.0);
   bool init();                   //initialise
   bool update();                 //update calculations
   float getAngle();               //get the updated estimate
   void setOffset(float offset); //set the offset for the returned angle
   bool isUpright();
   float getRpm();
   void reset();
   void calibrate();
   void adjustGyroConversion(float adjustment);
   float getGyroConversion();
   
   static float normalizeAngle(float angle, bool incNeg = false);               //convert to 2 pi 
};

#endif

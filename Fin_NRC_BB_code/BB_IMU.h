/************************************************************************/ /**
*@file
*@brief Contains the POV structures and classes prototypes.
***************************************************************************/
#ifndef BB_IMU
#define BB_IMU
 
#include "Arduino.h"
#include <Arduino_LSM6DS3.h>
#include <math.h>
//#include <SimpleKalmanFilter.h>
 
#define num_input 3
 
/***********************************************************************/ /**
*@class
*@brief This class manages the virtual screens used for animations and
*backgrounds of the display. Essentially it groups "stripe"s together to make
*images.
***************************************************************************/
class bb_imu
{
private:
   float AVG_gyro_vals[num_input]; //gX, gY, gZ
   float GYRO_vals[num_input];
 
   struct integrate_struct
   {
       float vals[3] = {0,0,0};
       int mid = 0;
       float integral = 0;
       float speed = 0;
       int * conversion;
   };
 
   unsigned long loop_start_time = micros(); //loop start time
   float delta_t = .00001; //time between measurements.
 
   //move to private...
   bool Get_raw(); //get raw values adjusted by average static error
   float integrate(integrate_struct &data, float new_val); //integrate values
   float make2pi(integrate_struct &data); //convert to 2 pi
  
   integrate_struct gyro_z;//struct that contains all data for the z gyroscope
 
public:
   bb_imu();
   bool init(); //initialise
   bool update(); // update calculations
   float Get_val(); //get the updated estimate
 
   int gyro_to_rad = 420;
 
};
 
 
 
#endif
 
 


/************************************************************************/ /**
*@file
*@brief Contains the POV structures and classes prototypes.
***************************************************************************/
#ifndef BB_IMU
#define BB_IMU
 
#include <Arduino.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "/home/joseph/Desktop/Robot/NRC/MotorSpeedController/Software/controller/Filter.cpp"
 
#define num_input 6
 
enum sensor
{
  gyroscope_x,
  gyroscope_y,
  gyroscope_z,
  accelerometer_x,
  accelerometer_y,
  accelerometer_z
};
 
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
  float offset = 0;
  Adafruit_ICM20649 IMU;
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp; //values not used but necessary for function
 
  //structure that contains all values and systems used for the
  //conditioning and integration of a single sensor
  struct integrate_struct
  {
     float vals[3] = {0, 0, 0};
     int mid = 0;
     float integral = 0;
     float speed = 0;
     int *conversion; //used to refer to a public int
     Filter rf = Filter(3); //set up a averaging low pass filter for the last three values
  };
 
  unsigned long loop_start_time = micros(); //loop start time
  float delta_t = .00001;                   //time between measurements.
 
  bool Get_raw();                                         //get raw values adjusted by average static error
  float integrate(integrate_struct &data, float new_val); //integrate values
  float make2pi(integrate_struct &data);                  //convert to 2 pi
 
  integrate_struct gyro_z; //struct that contains all data for the z gyroscope
 
public:
  bb_imu();
  ~bb_imu();
  bool init();                   //initialise
  bool update();                 //update calculations
  float Get_val();               //get the updated estimate
  void Set_offset(float offset); //set the offset for the returned angle
  bool Get_upright();
 
  int gyro_to_rad = 429;
  int accelerometer_range = ICM20649_ACCEL_RANGE_4_G;
  int gyroscope_range = ICM20649_GYRO_RANGE_500_DPS;
};
 
#endif

#include "BB_IMU.h" //?

bb_imu IMU_test;

void setup() {

  Serial.begin(9600);

  while(!Serial) {}
  
  Serial.println("Start");
  
  if(!IMU_test.init())
    while(true)
    {
      Serial.println("IMU setup failed");
      delay(500);
    }

  Serial.print("Accelerometer range set to: ");
  switch (IMU_test.accelerometer_range) 
  {
  case ICM20649_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20649_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20649_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  case ICM20649_ACCEL_RANGE_30_G:
    Serial.println("+-30G");
    break;
  }

  Serial.print("Gyro range set to: ");
  switch (IMU_test.gyroscope_range) 
  {
  case ICM20649_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }
    
  Serial.println("IMU setup fin");
}

void loop() {
    
  while(true)
  {

    IMU_test.update();

    Serial.print("Rotaion: ");    
    Serial.println(IMU_test.Get_val());
    Serial.print("Orientation: ");
    if(IMU_test.Get_upright())
    {
      Serial.println("Up");
    } 
    else   
    {
      Serial.println("Down");
    }
    
    delay(1);
  }
  
}

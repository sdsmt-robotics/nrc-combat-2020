#ifndef IMU_H
#define IMU_H

#include <Adafruit_ICM20649.h>

#include <SimpleKalmanFilter.h>

#define PI_2 2.0 * PI
#define DEG_2_RAD PI / 180
#define RAD_2_DEG 180 / PI

class IMU {
public:
  IMU();
  IMU(int cs, int sck, int miso, int mosi);
  IMU(int cs);
  bool init();
  void update();
  void calibrate(int num_readings = 1000);
  float getAngle();
  float getVelocity();
  void reset();

  void modifyDrift(float a);

private:
  float normalizeAngle(float angle);
  bool readIMU();
  float integrate(float deltaT);

  Adafruit_ICM20649 imu;

  SimpleKalmanFilter filter;

  sensors_event_t gyro;
  sensors_event_t accel;
  sensors_event_t temp;
  sensors_event_t mag;

  float angle = 0;

  float drift = 0;

  long last_update = 0;

  float last_value = 0;
  float last_filter_value = 0;

  int _cs_pin = 0;
  int _sck_pin = 0;
  int _miso_pin = 0;
  int _mosi_pin = 0;
};

#endif
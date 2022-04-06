#ifndef IMU_H
#define IMU_H

#include <Adafruit_ICM20649.h>
#include <SimpleKalmanFilter.h>

#include <SimpleKalmanFilter.h>

#define PI_2 2.0 * PI

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
  float getRawVelocity();
  void reset();

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
  float lastVal = 0; // Last value read from IMU
  float lastValFiltered = 0; // Last value read from IMU, filtered

  float gyroVals[3] = {0, 0, 0};
  int valsMidIdx = 0;

  int _cs_pin = -1;
  int _sck_pin = -1;
  int _miso_pin = -1;
  int _mosi_pin = -1;
};

#endif
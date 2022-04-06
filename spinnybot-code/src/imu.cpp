#include "imu.h"

IMU ::IMU() : filter(1, 1, 0.08) {
  _cs_pin = -1;
  _sck_pin = -1;
  _miso_pin = -1;
  _mosi_pin = -1;
}

IMU::IMU(int cs) : IMU() {
  _cs_pin = cs;
  _sck_pin = -1;
  _miso_pin = -1;
  _mosi_pin = -1;
}

IMU::IMU(int cs, int sck, int miso, int mosi) : IMU() {
  _cs_pin = cs;
  _sck_pin = sck;
  _miso_pin = miso;
  _mosi_pin = mosi;
}

bool IMU::init() {
  if (_cs_pin == -1) {
    if (!imu.begin_I2C()) {
      return false;
    }
  } else {
    if (_sck_pin == -1 && _miso_pin == -1 && _mosi_pin == -1) {
      if (!imu.begin_SPI(_cs_pin)) {
        return false;
      }
    } else {
      if (!imu.begin_SPI(_cs_pin, _sck_pin, _miso_pin, _mosi_pin)) {
        return false;
      }
    }
  }
  imu.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);
  calibrate();
  reset();
  return true;
}

void IMU::update() {
  unsigned long curTime;

  // get new raw values from the IMU
  curTime = micros();
  bool receivedVals = readIMU();

  // if there are new raw values then recalculate the speed
  if (receivedVals) {
    // update time since last update
    float deltaT = float(curTime - last_update) / 1000000;
    last_update = curTime;

    // integrate the gyroscope_z value
    integrate(deltaT);

    // Put into range [-PI, PI]
    angle = normalizeAngle(angle);
  }
}

bool IMU::readIMU() {
  bool readSuccess = true;

  // Attempt a read from the IMU
  readSuccess = imu.getEvent(&accel, &gyro, &temp);

  if (readSuccess) {
    // Add the new value
    last_value = gyro.gyro.z - drift;
    last_filter_value = filter.updateEstimate(last_value);
  }

  // imu was ready
  return readSuccess;
}

float IMU::integrate(float deltaT) {

  angle += (last_filter_value + last_value) / 2 * deltaT;
  last_value = last_filter_value;

  return angle;
}

float IMU::getAngle() { return angle; }

float IMU::getVelocity() { return last_filter_value; }

void IMU::calibrate(int num_readings) {
  drift = 0;
  for (int i = 0; i < num_readings; i++) {
    imu.getEvent(&accel, &gyro, &temp, &mag);
    drift += gyro.gyro.z;
    delay(1);
  }
  drift /= num_readings;
}

float IMU::normalizeAngle(float angle) {
  if (angle > PI) {
    do {
      angle -= PI_2;
    } while (angle > PI);
  } else {
    while (angle < -PI) {
      angle += PI_2;
    }
  }
  return angle;
}

void IMU::reset() { angle = 0.0; }

void IMU::modifyDrift(float a) { drift += a; }
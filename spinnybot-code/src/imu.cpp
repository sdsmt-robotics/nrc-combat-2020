#include "imu.h"

IMU::IMU() : filter(3) {}

bool IMU::init() {
  if (!imu.begin_SPI(22))
    // if (!imu.begin_I2C())
    return false;
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
    filter.filter((gyro.gyro.z - drift) * DEG_2_RAD);
  }

  // imu was ready
  return readSuccess;
}

float IMU::integrate(float deltaT) {
  // based off of simpsons 1/3 to find the approximate integral

  // Add the new value to the rolling array
  gyroVals[valsMidIdx] =
      filter.getFilteredVal(); // TODO: Using this filter seems a bit sus.
  if (valsMidIdx >= 2) {
    valsMidIdx = 0;
  }

  // compute simpsons 1/3 for the past three values
  angle +=
      ((gyroVals[0] + gyroVals[1] + gyroVals[2] + 3 * gyroVals[valsMidIdx]) /
       3) *
      deltaT;

  // update the new mid of the array
  ++valsMidIdx;

  return angle;
}

float IMU::getAngle() { return angle; }

float IMU::getVelocity() { return filter.getFilteredVal(); }

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
    do {
      angle += PI_2;
    } while (angle < -PI);
  }
  return angle;
}

void IMU::reset() {
  angle = 0.0;
  for (int i = 0; i < 3; i++)
    gyroVals[i] = 0;
  last_update = micros();
}
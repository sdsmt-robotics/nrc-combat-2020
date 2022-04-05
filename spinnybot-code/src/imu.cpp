#include "imu.h"

IMU ::IMU() : filter(1, 1, 0.08) {}

IMU::IMU(int cs) : IMU() {

    _cs_pin = cs;
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

/**
 * Read and save a value from the IMU. 
 * 
 * @return True if successful, false otherwise.
 */
bool IMU::readIMU() {
    bool readSuccess = true;

    // Attempt a read from the IMU
    readSuccess = imu.getEvent(&accel, &gyro, &temp);

    if (readSuccess) {
        // Add the new value
        lastVal = gyro.gyro.z - drift;
        lastValFiltered = filter.updateEstimate(lastVal /* * DEG_TO_RAD*/);
    }

    // imu was ready
    return readSuccess;
}

/**
 * Integrate the velocity value over the given time period to update the angle estimate.
 * 
 * @param deltaT - Time that has elapsed since the last sample
 * @return The angle estimate (in radians).
 */
float IMU::integrate(float deltaT) {
    // based off of simpsons 1/3 to find the approximate integral

    // Add the new value to the rolling array
    // gyroVals[valsMidIdx] = lastValFiltered;  // TODO: Using this filter seems a bit sus.
    // if (valsMidIdx >= 2) {
    //     valsMidIdx = 0;
    // }

    // // compute simpsons 1/3 for the past three values
    // angle += ((gyroVals[0] + gyroVals[1] + gyroVals[2] + 3 * gyroVals[valsMidIdx]) / 3) * deltaT;

    // // update the new mid of the array
    // ++valsMidIdx;
    static float lastVal = lastValFiltered;
    
    // Trapezoidal approximation
    angle += (lastValFiltered + lastVal) / 2 * deltaT;

    lastVal = lastValFiltered;

    return angle;
}

float IMU::getAngle() {
  return angle;
}

float IMU::getVelocity() {
    return lastValFiltered;
}

float IMU::getRawVelocity() {
    return lastVal;
}

void IMU::calibrate(int num_readings) {
    drift = 0;
    for (int i = 0; i < num_readings; i++) {
        imu.getEvent(&accel, &gyro, &temp, &mag);
        drift += gyro.gyro.z;
        delay(1);
    }
    drift /= num_readings;
}

/**
 * Normalize an angle to the range [-PI, PI].
 *
 * @param angle - The angle to normalize.
 * @return the normalized angle
 */
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

void IMU::reset() {
    angle = 0.0;
    for (int i = 0; i < 3; i++)
        gyroVals[i] = 0;
    last_update = micros();
}
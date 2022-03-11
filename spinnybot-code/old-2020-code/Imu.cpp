/************************************************************************/ /**
*@file
*@brief Get angle, rotation rate, and orientation from IMU.
***************************************************************************/
#include "Imu.h"


#define ICM_CS 32
#define ICM_SCK 33
#define ICM_MISO 25
#define ICM_MOSI 26

#define PI_2         2.0*PI
#define RADPS_2_RPM 60.0 / PI_2


/** ***************************************************************************
* @par Description:
* Constructor for the Imu class. This constructor basically sets the 
* the initial arrays to zero and assigns conversions to the relevant
* sensor structors.
*
* @param imuOffset - the angle offset in radians of the IMU (optional)
*****************************************************************************/
Imu::Imu(unsigned csPin, float imuOffset) {
    this->csPin = csPin;
    this->imuOffset = imuOffset;
}

/** ***************************************************************************
* @par Description:
* Initialises the class by calibrating the sensors and testing if the IMU 
* could be initialized.
*
* @returns true if successful and false otherwise.
*****************************************************************************/
bool Imu::init() {
  
    //test if the imu has started successfully
    if (!IMU.begin_SPI(csPin))
        return false;

    IMU.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);
    
    // Do a reset (initialize) of values
    reset();
    
    // Calibrate for gyro drift
    calibrate();

    return true;
}

/** ***************************************************************************
 * @brief Take some readings with the IMU to get average drift.
 * 
*****************************************************************************/
void Imu::calibrate() {
    //average some values from the IMU to use for calibration
    for (int i = 0; i < CALIBRATE_NUM_READINGS; i++) {
        IMU.getEvent(&accel, &gyro, &temp);

        gyroDrift += gyro.gyro.z;
        
        delay(CALIBRATE_READ_WAIT);
    }

    // Divide to get the average
    gyroDrift /= CALIBRATE_NUM_READINGS;
}

/** ***************************************************************************
* @par Description:
* Get the raw values for the IMU adjusted by their average error.
*
* @returns true if an update was available and false otherwise.
*****************************************************************************/
bool Imu::readImu() {
    int i;
    bool readSuccess = true;

    // Attempt a read from the IMU
    readSuccess = IMU.getEvent(&accel, &gyro, &temp);

    if (readSuccess) {
        // Add the new value
        angleRateFilter.filter((gyro.gyro.z - gyroDrift) * gyroToRad);
        accelZFilter.filter(accel.acceleration.z);
    }

    //imu was ready
    return readSuccess;
}

/** ***************************************************************************
* @par Description:
* Performs simpsons 1/3 integration on the last three data points without
* multiplying by delta t. The function doesn't return useful values until
* the first three values are passed in and you must pass in at least three
* new values to completely clear the function.
*
* @param [in] new_val : the newest value to be used in the estimation.
* @param [in/out] data : a structure that contains all relevant data for 
* the estimation.
*****************************************************************************/
float Imu::integrateGyro(float deltaT)
{
    //based off of simpsons 1/3 to find the approximate integral

    // Add the new value to the rolling array
    gyroVals[valsMidIdx] = angleRateFilter.getFilteredVal();  // TODO: Using this filter seems a bit sus.
    if (valsMidIdx >= 2) {
        valsMidIdx = 0;
    }

    //compute simpsons 1/3 for the past three values
    angle += ((gyroVals[0] + gyroVals[1] + gyroVals[2] + 3 * gyroVals[valsMidIdx]) / 3) * deltaT;
    
    //update the new mid of the array
    ++valsMidIdx;
}

/** ***************************************************************************
 * @brief Reset the current angle estimate and the recent angle rate values.
 * 
*****************************************************************************/
void Imu::reset() {
    angleRateFilter.reset();
    accelZFilter.reset(-1); // Assume upright
    angle = 0.0;
    for (int i = 0; i < 3; i++)
        gyroVals[i] = 0;
    lastUpdateTime = micros();
}

/** ***************************************************************************
* @par Description:
* Update the estimation for all IMU values.
*
* @returns true for a successful update, false otherwise.
*****************************************************************************/
bool Imu::update()
{
    unsigned long curTime;

    //get new raw values from the IMU
    curTime = micros();
    bool receivedVals = readImu();

    //if there are new raw values then recalculate the speed
    if (receivedVals)
    {
        //update time since last update
        float deltaT = float(curTime - lastUpdateTime) / 1000000;
        lastUpdateTime = curTime;

        //integrate the gyroscope_z value
        integrateGyro(deltaT);

        // Put into range [0, 2PI]
        angle = normalizeAngle(angle);
    }

    return receivedVals;
}

/** ***************************************************************************
 * @brief Adjust the value used to convert from gyro values to rad/s.
 * 
 * @param adjustment - Amount by which to change the conversion value.
*****************************************************************************/
void Imu::adjustGyroConversion(float adjustment) {
    gyroToRad += adjustment;
}

/** ***************************************************************************
 * @brief Get the value used to convert from gyro values to rad/s.
 * 
 * @return The conversion value.
*****************************************************************************/
float Imu::getGyroConversion() {
    return gyroToRad;
}

/** ***************************************************************************
 * @brief Convert an angle to the normalized range [0, 2PI] (or [-2PI, 2PI]).
 * 
 * @param angle - Angle in radians
 * @param incNeg - Use the expanded range [-2PI, 2PI].
 * @return Angle normalized to the range [0, 2PI]
*****************************************************************************/
float Imu::normalizeAngle(float angle, bool incNeg) {
    // Repeatedly add or subtract a full turn until in acceptable range
    if (angle > PI_2) {
        do {
            angle -= PI_2;
        } while (angle > PI_2);
    } else {
        while ((incNeg && angle < PI_2) || (!incNeg && angle < 0)) {
            angle += PI_2;
        }
    }

  return angle;
}

/** ***************************************************************************
* @par Description:
* Return a float of the estimated bb angle. 
*
* @returns float of the estimated bb angle.
*****************************************************************************/
float Imu::getAngle() {
    return normalizeAngle(angle - imuOffset);
}

/** ***************************************************************************
 * @brief Get the current speed in RPM.
 * 
 * @return The current speed.
*****************************************************************************/
float Imu::getRpm() {
  return angleRateFilter.getFilteredVal() * RADPS_2_RPM;
}

/** ***************************************************************************
 * @brief Sets the angle offset for angle readings. Used to account for IMU not
 * being oriented with "straight" on the bot.
 * 
 * @param offset - new angle offset value in radians. 
*****************************************************************************/
void Imu::setOffset(float offset)
{
    imuOffset = normalizeAngle(offset, true);
}

/** ***************************************************************************
* @par Description:
* Tests the IMU to see if the bot is upright
*
* @returns true if the bot is upright or false if the bot is upside down
*****************************************************************************/
bool Imu::isUpright() {
    return accelZFilter.getFilteredVal() > 0.0;
}

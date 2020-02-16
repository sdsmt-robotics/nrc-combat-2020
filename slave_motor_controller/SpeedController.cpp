/* 1/18/2020
 * Christian Weaver & Samuel Ryckman
 *
 * SpeedController class.
 * This file provides function definitions to regulate the power outputted to a motor to maintain a given speed.
 */

#include "SpeedController.h"


SpeedController::SpeedController(InitSpeedControllerValues initVals, AS5134* encoder)
{
    this->encoder = encoder;
    this->encoder->init();
    this->maxOutput = initVals.maxOutput;
    this->minOutput = initVals.minOutput;
    this->maxOutCap = initVals.maxOutCap;
    this->minOutCap = initVals.minOutCap;
    this->maxInput = initVals.maxInput;
    this->minInput = initVals.minInput;
}


int SpeedController::motorSpeedToPower(int desiredMotorSpeed)
{
    int motorSpeed = getSpeed();
    motorSpeed = filterMotorSpeed(motorSpeed);
    int motorPower = computeMotorPower(motorSpeed, desiredMotorSpeed);

    return motorPower;
}


int SpeedController::computeMotorPower(int motorSpeed, int desiredSpeed)
{
    static long lastTime = 0;
    static int lastError = 0;
    long curTime = micros();

    //Define PID constants
    const float kp = (float)abs(maxOutput - minOutput) / abs(maxInput - minInput);
    const float ki = 1;
    const float kd = 0.004;

    int target = desiredSpeed;//1000;

    static float integral = 0;
    float derivative = 0;

    int output = 0;
    int error = 0;
    float elapsedTime;


    // Skip if first loop so we can get initial values
    if (lastTime == 0) {
        lastTime = micros();
        lastError = target - motorSpeed;
        for(int i = 0; i < 5; i++)
        {
          filterMotorSpeed(motorSpeed);
        }
        return (maxOutput + minOutput) / 2;
    }

    //update the time
    elapsedTime = (curTime - lastTime) / 1000000.0;
    lastTime = curTime;

    //get the error
    error = target - motorSpeed;

    //Get the integral of the error
    integral += (float)error * elapsedTime;

    //Get the derivative of the error
    derivative = (float)(error - lastError) / elapsedTime;

    //calculate the output
    output = kp * error + ki * integral + kd * derivative;

    //cap the output and return
    return constrain(output, minOutCap, maxOutCap);
}


int SpeedController::filterMotorSpeed(int curMotorSpeed)
{
    static int pastVals[] = { 0, 0, 0, 0, 0 };
    const int numVals = 5;
    //static int average = 0;
    static int curIndex = 0;
    long total = 0;

    //add the new value
    pastVals[curIndex] = curMotorSpeed;

    //update position in the rolling array
    curIndex++;
    if (curIndex >= numVals) {
        curIndex = 0;
    }

    //get the total
    for (int i = 0; i < numVals; i++) {
        total += pastVals[i];
    }

    //return the average
    return total / numVals;
}


int SpeedController::getSpeed() {
    static unsigned long curTime, lastTime = micros();
    static int curPos, lastPos = encoder->readMultiTurnAngle();
    static int curSpeed;

    //Calculate the speed based on the current position
    curPos = encoder->readMultiTurnAngle();
    curTime = micros();
    curSpeed = long(curPos - lastPos) * 166667 / (curTime - lastTime); //rpm
    //curSpeed = (curPos - lastPos); //rpm

    encoder->resetCounter();
    lastPos = encoder->readMultiTurnAngle();
    lastTime = micros();

    curMotorSpeed1 = curSpeed;

    return curSpeed;
}

int SpeedController::getSpeed1()
{
  return curMotorSpeed1;
}

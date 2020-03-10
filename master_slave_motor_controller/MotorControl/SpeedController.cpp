/* 1/18/2020
* Christian Weaver & Samuel Ryckman
*
* SpeedController class.
* This file provides function definitions to regulate the power outputted to a motor to maintain a given speed.
* 
* void setPIDConsts(float kp, float ki, float kd) - set the PID constants for the control
* void setOutputLimits(int min, int max) - set the min and max output limits.
* void setTarget(int targetSpeed) - set the target speed for the controller.
* int getTarget() - get teh target speed from the controller
* int calcOutput(int curSpeed) - calculate the motor power given the current motor speed
*/
 
#include "SpeedController.h"


/**
 * @brief Constructor for the class.
 */
SpeedController::SpeedController() : outputFilter(5)/*outputFilter(25, 25, 0.01)*/ {
    lastTime = micros();
    lastError = 0;
    integral = 0;
    
    targetSpeed = 0;

    //constrain the integral so it doesn't run away.
    maxIntegral = maxOutput / ki * 4;
    minIntegral = minOutput / ki * 4;
}
 
/**
 * @brief Compute the output (power) given the current input (speed).
 * 
 * @param curSpeed - current speed of the motor.
 */
int SpeedController::calcOutput(int curSpeed) {
  long curTime = micros();
  int error = 0;
  int output = 0;
  float elapsedTime;

  //update the time
  elapsedTime = (curTime - lastTime) / 1000000.0;

  //get the error
  error = targetSpeed - curSpeed;

  //Get the integral of the error
  integral += (float)error * elapsedTime;
  if (integral < minIntegral) {
    integral = minIntegral;
  } else if (integral > maxIntegral) {
    integral = maxIntegral;
  }

  //Get the derivative of the error
  derivative = (float)(error - lastError) / elapsedTime;

  //calculate the output
  output = kp * error + ki * integral + kd * derivative;

  //save for next calculation
  lastError = error;
  lastTime = curTime;

  //return the filtered and caped output
  //return outputFilter.updateEstimate(constrain(output, minOutput, maxOutput));
  return outputFilter.filter(constrain(output, minOutput, maxOutput));
  //return constrain(output, minOutput, maxOutput);
}

/**
 * @brief set the PID constants for the controller.
 * 
 * @param kp - proportional constant.
 * @param ki - integral constant.
 * @param kd - derivative constant.
 */
void SpeedController::setPIDConsts(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

/**
 * @brief set the output limits for the controller.
 * 
 * @param min - minimum output value.
 * @param max - maximum output value.
 */
void SpeedController::setOutputLimits(int min, int max) {
  this->minOutput = min;
  this->maxOutput = max;
}

/**
 * @brief set the target speed for the controller.
 * 
 * @param targetSpeed - the speed we are trying to maintain.
 */
void SpeedController::setTarget(int targetSpeed) {
  this->targetSpeed = targetSpeed;
}


/**
 * @brief Get the target speed for the controller.
 * 
 * @return target speed for the controller.
 */
int SpeedController::getTarget() {
  return targetSpeed;
}

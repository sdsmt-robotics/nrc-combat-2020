/* 1/18/2020
* Christian Weaver & Samuel Ryckman
*
* SpeedController class.
* This file provides function definitions to regulate the power outputted to a motor to maintain a given speed.
*/
 
#include "SpeedController.h"
 
SpeedController::SpeedController( AS5134 *encoder)
{
   this->encoder = encoder;
   this->encoder->init();
 
}
 
int SpeedController::motorSpeedToPower(int desiredMotorSpeed)
{
   //Compute the motor power by comparing the filtered motor
   //speed and the desired motor speed
   return computeMotorPower(filterMotorSpeed(getSpeed()), desiredMotorSpeed);
}
 
int SpeedController::computeMotorPower(int motorSpeed, int desiredSpeed)
{
   //calculate and store time info
   static long lastTime = 0;
   static int lastError = 0;
   long curTime = micros();
   float elapsedTime;
 
   //Define general variables
   static float integral;
   float derivative = 0;
   int output = 0;
   int error = 0;
 
   // Skip if first loop to set up initial values
   if (lastTime == 0)
   {
       lastTime = micros();
       integral = 0;
       lastError = desiredSpeed - motorSpeed;
       for (int i = 0; i < 5; i++)
       {
           filterMotorSpeed(motorSpeed);
       }
       return (maxOutput + minOutput) / 2;
   }
 
   //update the time
   elapsedTime = (curTime - lastTime) / 1000000.0;
   lastTime = curTime;
 
   //get the error
   error = desiredSpeed - motorSpeed;
 
   //Get the integral of the error
   integral += float(error) * elapsedTime;
 
   //constrain acumulator
   constrain(integral,minAcumulator,maxAcumulator);
 
   //Get the derivative of the error
   derivative = float((error) - lastError) / elapsedTime;
 
   //calculate the output
   output = kp * error + ki * integral + kd * derivative;
 
   //cap the output and return
   return constrain(output, minOutCap, maxOutCap);
}
 
int SpeedController::filterMotorSpeed(int curMotorSpeed)
{
   static int pastVals[] = {0, 0, 0, 0, 0, 0, 0};
   const int numVals = 7;
   static int curIndex = 0;
   static long total = 0;
 
   //subtract the outdated value
   total -= pastVals[curIndex];
 
   //add the new value to the array
   pastVals[curIndex] = curMotorSpeed/numVals;
 
   //add the new value to the total
   total +=pastVals[curIndex];
 
   //update position in the rolling array
   curIndex++;
   if (curIndex >= numVals)
   {
       curIndex = 0;
   }
 
   return total;
}
 
int SpeedController::getSpeed()
{
   static unsigned long curTime, lastTime  = micros();
   static int curPos, lastPos  = encoder->readMultiTurnAngle();
 
   //Calculate the speed based on the current position
   curPos = encoder->readMultiTurnAngle();
   curTime = micros();
   curMotorSpeed1 = long(curPos - lastPos) * 166667 / (curTime - lastTime); //rpm
 
   //reset counter and reset set last pos and time to new values
   encoder->resetCounter();
   lastPos = encoder->readMultiTurnAngle();
   lastTime = micros();
 
   return curMotorSpeed1;
}
 
int SpeedController::getSpeed1()
{
   return curMotorSpeed1;
}

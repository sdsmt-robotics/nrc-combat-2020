/* 1/18/2020
 * Christian Weaver & Samuel Ryckman
 *
 * Header for the SpeedController class
 */

#ifndef SPEEDCONTROLLER
#define SPEEDCONTROLLER

//#include <cmath>
#include "AS5134.h"
#include "Arduino.h"

using namespace std;                                                           

class SpeedController
{
public:
    SpeedController(AS5134 * encoder);
    int getSpeed1();
        
    int motorSpeedToPower(int desiredSpeed);


private:
    AS5134* encoder;
    const int maxOutput = 255;    //max output value
    const int minOutput = -255;   //min output value
    const int maxOutCap = 40;     //max output we will send
    const int minOutCap = -40;    //min output we will send
    const int maxInput = 18000;  //max possible input value
    const int minInput = -18000; //min possible input value
    const float maxAcumulator = 70;
    const float minAcumulator = -70;
    int curMotorSpeed1;

    //Define PID constants
    const float kp = float(abs(maxOutput - minOutput)) / abs(maxInput - minInput);
    const float ki = 1;
    const float kd = 0.004;

    int computeMotorPower(int motorSpeed, int desiredMotorSpeed);
    int filterMotorSpeed(int curMotorSpeed);
    int getSpeed();
};




#endif

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
    float getSpeed1();
        
    int motorSpeedToPower(int desiredSpeed);


private:
    AS5134* encoder;
    const int maxOutput = 255;    //max output value
    const int minOutput = -255;   //min output value
    const int maxOutCap = 120;     //max output we will send
    const int minOutCap = -120;    //min output we will send
    const int maxInput = 18000;  //max possible input value
    const int minInput = -18000; //min possible input value
    const float maxAcumulator = 300;
    const float minAcumulator = -300;
    float curMotorSpeed1;

    //Define PID constants
    const float kp = 1;//float(abs(maxOutput - minOutput)) / abs(maxInput - minInput);
    const float ki = 1.5;
    const float kd = 0.004;

    int computeMotorPower(int motorSpeed, int desiredMotorSpeed);
    int filterMotorSpeed(int curMotorSpeed);
    float getSpeed();
};




#endif

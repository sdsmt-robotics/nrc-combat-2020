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

const int numSampledSpeeds = 5;

//This concisely stores the initialization values for the SpeedController constructor
struct InitSpeedControllerValues
{
    int maxOutput = 255;    //max output value
    int minOutput = -255;   //min output value
    int maxOutCap = 40;     //max output we will send
    int minOutCap = -40;    //min output we will send
    int maxInput = 18000;  //max possible input value
    int minInput = -18000; //min possible input value
};


class SpeedController
{
public:
    SpeedController(InitSpeedControllerValues initVals, AS5134 * encoder);
    int getSpeed1();
        
    int motorSpeedToPower(int desiredSpeed);


private:
    AS5134* encoder;
    int maxOutput;  //max output value
    int minOutput;  //min output value
    int maxOutCap;  //max output we will send
    int minOutCap;  //min output we will send
    int maxInput;   //max possible input value
    int minInput;   //min possible input value
    int curMotorSpeed1;

    int computeMotorPower(int motorSpeed, int desiredMotorSpeed);
    int filterMotorSpeed(int curMotorSpeed);
    int getSpeed();
};




#endif

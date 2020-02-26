/* 1/18/2020
 * Christian Weaver & Samuel Ryckman
 *
 * Header for the SpeedController class
 */

#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include "Arduino.h"
#include "Filter.h"
                                                 

class SpeedController {
public:
    SpeedController();
    
    void setPIDConsts(float kp, float ki, float kd);
    void setOutputLimits(int min, int max);
    void setTarget(int targetSpeed);
    int getTarget();

    int calcOutput(int curSpeed);

private:
    Filter outputFilter;  //filter to smooth output values

    int targetSpeed;  //speed we are trying to maintain

    //output limits
    int maxOutput = 255;   //max output we will send
    int minOutput = 0;     //min output we will send

    //values from the most recent speed calculation
    long lastTime;
    int lastError;

    //integral of the error. Have to save between calculations since it accumulates.
    float integral;

    //Define PID constants
    float kp = 0.08;    //guess: float(abs(maxOutput - minOutput)) / abs(maxInput - minInput);
    float ki = 1;
    float kd = 0.02;
};




#endif

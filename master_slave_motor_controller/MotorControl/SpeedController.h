/* 1/18/2020
 * Christian Weaver & Samuel Ryckman
 *
 * Header for the SpeedController class
 */

#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include "Arduino.h"
#include "Filter.h"
//#include <SimpleKalmanFilter.h>
                                                 

class SpeedController {
public:
    SpeedController();
    
    void setPIDConsts(float kp, float ki, float kd);
    void setOutputLimits(int min, int max);
    void setTarget(int targetSpeed);
    int getTarget();

    int calcOutput(int curSpeed);
    
    //integral of the error. Have to save between calculations since it accumulates.
    float integral;
    float derivative = 0;
    int lastError;

private:
    Filter outputFilter;  //filter to smooth output values
    //SimpleKalmanFilter outputFilter;

    int targetSpeed;  //speed we are trying to maintain

    //output limits
    int maxOutput = 255;   //max output we will send
    int minOutput = 0;     //min output we will send
    float maxIntegral;  //max value for the integral
    float minIntegral;  //min value for the integral

    //values from the most recent speed calculation
    long lastTime;


    //Define PID constants
    float kp = 1;    //guess: float(abs(maxOutput - minOutput)) / abs(maxInput - minInput);
    float ki = 1;
    float kd = 1;
};




#endif

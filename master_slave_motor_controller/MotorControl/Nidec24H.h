/* 
 * Header for the Nidec 24H brushless motor.
 */

#ifndef AS5134_H
#define NIDEC24H_H

#include "Arduino.h"
//#include "Filter.h"
#include <SimpleKalmanFilter.h>

/**
 * @brief Direction constants to set motor direction.
 */
enum Direction { FORWARD, REVERSE };

class Nidec24H {
public:
    Nidec24H(int pwmPin, int dirPin, int brakePin, int fgPin = -1);
  
    void init();
    void setPower(int power);
    int getPower();
    void brake();
    void setDirection(Direction invertDir);
    Direction getDirection();
    uint16_t getSpeed();
    int getFilteredSpeed();

    int ticks = 0;
    unsigned long lastInterval = 0; //last fg time interval

private:
    void updateSpeed();
    int estimateSpeed();
    bool encoderTimedOut();

    //Static stuff for interrupt handling
    static void isr0();
    static void isr1();
    static void isr2();
    static void isr3();
    static Nidec24H* instance0;
    static Nidec24H* instance1;
    static Nidec24H* instance2;
    static Nidec24H* instance3;

    int pwmPin, dirPin, brakePin, fgPin;
    Direction direction = FORWARD;
    int power;
    uint16_t speed; //current speed of the motor in rpm
    uint16_t filteredSpeed; //current speed of the motor in rpm
    unsigned long fgTime; //time in microseconds of the last encoder tick
    bool readTimedOut = false;  //track if the last encoder reading is too old
    //Filter speedFilter;  //filter for the motor speed

    /*
     SimpleKalmanFilter(e_mea, e_est, q);
     e_mea: Measurement Uncertainty 
     e_est: Estimation Uncertainty 
     q: Process Noise
     */
    SimpleKalmanFilter speedFilter;
};

#endif

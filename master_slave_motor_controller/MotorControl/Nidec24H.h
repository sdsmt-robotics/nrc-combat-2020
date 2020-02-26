/* 
 * Header for the Nidec 24H brushless motor.
 */

#ifndef AS5134_H
#define NIDEC24H_H

#include "Arduino.h"
#include "Filter.h"

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
    int getSpeed();
    int getFilteredSpeed();

private:
    void updateSpeed();

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
    int speed; //current speed of the motor in rpm
    long fgTime; //time in microseconds of the last encoder tick
    Filter speedFilter;  //filter for the motor speed

};

#endif

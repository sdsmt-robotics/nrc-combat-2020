/* 
 * Header for the Nidec 24H brushless motor.
 */

#ifndef AS5134_H
#define NIDEC24H_H

#include "Arduino.h"

/**
 * @brief Direction constants to set motor direction.
 */
enum Direction { FORWARD, REVERSE };

class Nidec24H {
public:
    Nidec24H(int pwmPin, int dirPin, int brakePin);
  
    void init();
    void setPower(int power);
    void brake();
    void setDirection(Direction invertDir);
    Direction getDirection();

private:
    int pwmPin, dirPin, brakePin;
    Direction direction = FORWARD;
};

#endif
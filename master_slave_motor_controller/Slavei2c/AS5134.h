/* 1/10/2020
 * Samuel Ryckman
 * 
 * Header for the Massmind AS5134 class.
 */

#ifndef AS5134_H
#define AS5134_H

#include "Arduino.h"

class AS5134
{
public:
    AS5134(int dioPin, int csPin, int clkPin);
  
    void init();
    int readCounter();
    void resetCounter();
    int readAngle();
    long readMultiTurnAngle();
    void setLowPowerMode(bool enable = true);
    bool getLockAdc();
  
private:
    uint16_t getData(int command);
    void setData(int command, uint16_t data);
    uint16_t transmit(int command, bool sendMode, uint16_t data = 0);
    
    int dioPin, csPin, clkPin;
};

#endif
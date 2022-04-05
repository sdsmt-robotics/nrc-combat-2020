/* 1/10/2020
 * Samuel Ryckman
 * 
 * Header for the Massmind AS5134 class.
 */

#ifndef AS5134_H
#define AS5134_H

#include <Arduino.h>

enum {
    WRITE_OTP = 31,
    PROG_OTP = 25,
    WRITE_CONFIG = 23,
    SET_COUNTER = 20,
    EN_PROG = 16,
    READ_OTP = 15,
    READ_ANA = 9,
    RD_MT_COUNTER = 4,
    RD_ANGLE = 0,
} CMD;

enum {
    SISO_SIMPLE_DATAWIDTH = 16,
    SISO_EXTENDED_DATAWIDTH = 62
} SISO_MODE_DATAWIDTH;

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

    uint16_t getData(int command);
    uint64_t getData(int command,uint8_t len);
    void setData(int command, uint16_t data);
    void setData(uint8_t command, uint64_t data,uint8_t len);
  
private:
    uint16_t transmit(int command, bool sendMode, uint16_t data = 0);
    uint64_t transmitEx(uint8_t command, bool sendMode, uint64_t data=0, uint8_t length=16);
    int dioPin, csPin, clkPin;
};

#endif
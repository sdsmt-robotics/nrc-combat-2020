/* 
 * Header for the Nidec24hController class.
 */

#ifndef NIDEC_24H_CONTROLLER_H
#define NIDEC_24H_CONTROLLER_H

#include "Arduino.h"
#include <SPI.h>

class Nidec24hController {
public:
    Nidec24hController(SPIClass &spi, int ssPin);
  
    void init();

    void setSpeed(int speed);
    void setPower(int power);
    void brake();
    void invertDirection(bool invertDir);
    void setTimeout(unsigned timeout);

    int getSpeed();
    int getPower();
    int getTargetSpeed();

private:
    SPIClass &spi;
    int ssPin;

    const int TRANSFER_BYTE_DELAY = 80;
    const unsigned long MIN_SEND_INTERVAL = 150; // Minimum time between communicaitons in us.
    unsigned long lastCommunication = 0;

    int getVal(uint8_t command);
    void sendVal(uint8_t command, int value);
    int communicate(uint8_t command, int sendVal);
    
};

#endif

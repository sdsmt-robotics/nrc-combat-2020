// 1/7/2020
// Samuel Ryckman
//
// Class for the LY530AL and LPR530AL gyro sensor pair. 


#ifndef _L530AL_H_
#define _L530AL_H_

#include "Arduino.h"


class L530AL {
    public:
        L530AL();
        L530AL(uint8_t xPin, uint8_t yPin, uint8_t zPin);

        // getters for values
        void getVals(uint16_t &x, uint16_t &y, uint16_t &z);
        uint16_t getX();
        uint16_t getY();
        uint16_t getZ();

    private:
        //Anolog pins for the devices. Defaults are in the cpp file.
        uint8_t xPin;
        uint8_t yPin;
        uint8_t zPin;
};

#endif /* _L530AL_H_ */

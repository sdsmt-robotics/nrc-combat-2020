/* 1/10/2020
 * Samuel Ryckman
 * 
 * Massmind AS5134 rotary encoder class.
 * This class provides functions for interfacing with the encoder using 3 pin serial mode.
 * 
 * AS5134(dioPin, csPin, clkPin) - constructor. Provide the IO pin nums for communications.
 * init() - initialize the communications pins. Call in the Arduino setup() function.
 * readCounter() - read the multi-turn counter.
 * resetCounter() - reset the multi-turn counter to zero.
 * readAngle() - get the absolute angle value.
 * readMultiTurnAngle() - get the total angle value from the starting position.
 */
#include "AS5134.h"

//Define some commands
#define RD_ANGLE 0
#define RD_COUNTER 4
#define SET_COUNTER 20
#define WRITE_CONFIG 23

/** Constructor for the class.
*/
AS5134::AS5134(int dioPin, int csPin, int clkPin)
    : dioPin(dioPin),
      csPin(csPin),
      clkPin(clkPin)
{ }


/**
* Initialize the pins.
*/
void AS5134::init() {
    pinMode(csPin,  OUTPUT);
    pinMode(clkPin, OUTPUT);
    pinMode(dioPin, INPUT);
    
    digitalWrite(csPin, LOW);
    digitalWrite(clkPin, LOW);
}


/**
* Read the multi-turn counter.
* 
* @return the number of complete rotations.
*/
int AS5134::readCounter() {
    return getData(RD_COUNTER) >> 7;
}


/**
* Reset the multi-turn counter to 0.
*/
void AS5134::resetCounter() {
    setData(SET_COUNTER, 0);
}


/**
* Read the absolute angle value.
* 
* @return the absolute angle value.
*/
int AS5134::readAngle() {
    const int ANGLE_MASK = 0b111111111;
    
    return getData(RD_ANGLE) & ANGLE_MASK;
}


/**
* Read the total angle turned from original zero position.
* 
* @return the total angle value.
*/
long AS5134::readMultiTurnAngle() {
    //Read the values. Read turns twice to make sure we didn't loop over.
    int turns1 = readCounter();
    int angle = readAngle();
    int turns2 = readCounter();
    
    //If we looped around between reading turns1 and angle, have to use second turn count and re-read angle
    //This is rare, but does happen sometimes.
    if (turns1 != turns2) {
        angle = readAngle();
        turns2 = readCounter();  //re-read this just for timing consistancy
    }

    if( turns2 > 255)
    {
      turns2 -= 512; 
    }

    return (long)turns2 * 360 + angle;
}

/**
* Put the encoder to sleep. Angle value will be locked until taken out of sleep mode.
* 
* @param enable - Set whether sleep mode is on or off.
*/
void AS5134::setLowPowerMode(bool enable = true) {
    setData(WRITE_CONFIG, uint16_t(enable) << 15);
}

/**
* Status of the angle tracking ADC. Check whether it is locked on the magnet angle.
* 
* @return the status of the lock.
*/
bool AS5134::getLockAdc() {
    return getData(RD_ANGLE) >> 15;
}

/**
* Data transmission with the encoder for reading data
* 
* @param command - Data transmission command.
* @return the data received.
*/
uint16_t AS5134::getData(int command) {
    return transmit(command, false);
}

/**
* Data transmission with the encoder for sending data
* 
* @param command - Data transmission command.
* @param data - Data to send to the encoder.
*/
void AS5134::setData(int command, uint16_t data) {
    transmit(command, true, data);
}

/**
* Data transmission with the encoder for sending or reading data
* 
* @param command - Data transmission command.
* @param sendMode - Set whether we are sending or reading data.
* @param data - Data to send to the encoder.
* @return the data received if in read mode.
*/
uint16_t AS5134::transmit(int command, bool sendMode, uint16_t data = 0) {
    //NOTE: Delays have been removed from this. Max clock frequency of the encoder is 6MHZ. 
    // Might need to slow this down if there are problems.
    
    // Start the transmission
    digitalWrite(csPin,LOW);
    digitalWrite(clkPin,LOW);
    digitalWrite(csPin,HIGH);
    delayMicroseconds(1);

    //send the command
    pinMode(dioPin, OUTPUT);
    for(int bitNum = 4; bitNum >= 0; bitNum--) {
        //tick clock low
        digitalWrite(clkPin,LOW);
        //delayMicroseconds(1);

        //send the bit
        digitalWrite(dioPin, ((command >> bitNum) & 1));

        //tick clock high
        digitalWrite(clkPin,HIGH);
        //delayMicroseconds(1);
    }

    //read/write the data
    if (!sendMode) {
        pinMode(dioPin, INPUT);
    }
    for(int bitNum=15; bitNum >= 0; bitNum--) {
        //tick the clock LOW
        digitalWrite(clkPin,LOW);
        //delayMicroseconds(1);
        
        //send the bit if in send mode
        if (sendMode) {
            digitalWrite(dioPin, ((data >> bitNum) & 1));
        }
        
        //tick the clock HIGH
        digitalWrite(clkPin,HIGH);
        //delayMicroseconds(1);

        //Add the bit to the response if in read mode
        if (!sendMode) {
            if(digitalRead(dioPin)) {
              data += (1 << (bitNum));
            }
        }
    }

    //End the transmission
    digitalWrite(clkPin, LOW);
    digitalWrite(csPin, LOW);

    return data;
}

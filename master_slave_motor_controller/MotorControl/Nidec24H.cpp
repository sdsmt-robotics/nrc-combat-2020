/* 
 * Class for Nidec 24H brushless motor.
 * This class provides functions for interfacing with the motor.
 * 
 * init() - initialize the motor class.
 * setPower(int power) - set the power level for the motor
 * brake() - set the motor to brake.
 * setDirection(Direction invertDir) - set the default the motor direction.
 * getDirection() - get the motor direction
 * 
 * Note: this class is only set up to support four encoders. More is possible, 
 * but they will have to be added.
 */

#include "Nidec24H.h"

// for use by ISR glue routines
Nidec24H * Nidec24H::instance0;
Nidec24H * Nidec24H::instance1;
Nidec24H * Nidec24H::instance2;
Nidec24H * Nidec24H::instance3;

/**
 * @brief Constructor for the class.
 * 
 * @param pwmPin - pin to control speed using PWM.
 * @param dirPin - pin to toggle motor direction.
 * @param brakePin - pin to brake to toggle motor braking.
 * @param fgPin - digital pin number for reading encoder ticks. Must be an inturrupt pin.
 */
Nidec24H::Nidec24H(int pwmPin, int dirPin, int brakePin, int fgPin) 
    : pwmPin(pwmPin), dirPin(dirPin), brakePin(brakePin), fgPin(fgPin), speedFilter(6) {
}

/**
 * @brief Initialize the motor.
 */
void Nidec24H::init() {
    //Setup control pins
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(brakePin, OUTPUT);
    
    //Set up the encoder pin if given
    if (fgPin != -1) {
        int intNum = digitalPinToInterrupt(fgPin);
        pinMode(fgPin, INPUT_PULLUP);

        //We can't pass a class method into the attachInterrupt function. So, 
        //we instead track instances and have dedicated ISRs to forward the 
        //interrupt on to the actual handler function of the given instance.
        switch (intNum) {
        case 0:
            instance0 = this;
            attachInterrupt(intNum, Nidec24H::isr0, RISING);
            break;
        
        case 1:
            instance1 = this;
            attachInterrupt(intNum, Nidec24H::isr1, RISING);
            break;
        
        case 2:
            instance2 = this;
            attachInterrupt(intNum, Nidec24H::isr2, RISING);
            break;
        
        case 3:
            instance3 = this;
            attachInterrupt(intNum, Nidec24H::isr3, RISING);
            break;
        
        default:
            break;
        }


        //initialize the timer
        fgTime = micros();
    }

    //Initialize the power
    setPower(0);
}

/**
 * @brief set the motor to run in the given direction
 * 
 * @param direction - new direction for the motor to run
 */
void Nidec24H::setDirection(Direction direction) {
    this->direction = direction;
}

/**
 * @brief Get the current running direction for the motor.
 * 
 * @return Direction of the motor.
 */
Direction Nidec24H::getDirection() {
    return direction;
}

/**
 * @brief Run the motor at the given value (reverse is allowed).
 * 
 * @param power - the power level for the motor in the range 255 to -255.
 */
void Nidec24H::setPower(int power) {
    this->power = power;

    //stop the brake
    digitalWrite(brakePin, HIGH);

    //Get the desired direction.
    Direction runDirection = (power > 0 ? FORWARD : REVERSE);

    //Figure out which direction to run 
    if (runDirection == this->direction) {
        digitalWrite(dirPin, LOW);
    } else {
        digitalWrite(dirPin, HIGH);
    }
    
    //set the power
    analogWrite(pwmPin, constrain(abs(255 - power), 0, 255));
}

/**
 * @brief Get the current power level the motor is running at.
 * 
 * @return the current power level for the motor.
 */
int Nidec24H::getPower() {
    return power;
}

/**
 * @brief Engage the brake.
 */
void Nidec24H::brake() {
    //set to some power
    analogWrite(pwmPin, 50);

    //Turn on the brake
    digitalWrite(brakePin, LOW);
}

/**
 * @brief Get the current speed the motor is running at in RPMs.
 * 
 * @return the current speed of the motor.
 */
int Nidec24H::getSpeed() {
  return speed;
}

/**
 * @brief Get the filtered current speed the motor is running at in RPMs.
 * 
 * @return the filtered current speed of the motor.
 */
int Nidec24H::getFilteredSpeed() {
  return speedFilter.getFilteredVal();
}


/**
   @brief Called on encoder tick. Update the speed.
*/
void Nidec24H::updateSpeed() {
    speed = 10000000.0 / (micros() - fgTime);
    fgTime = micros();
    speedFilter.filter(speed);
}

/**
 * @brief Handle interrupt 0. Forward it to stored instance 0.
 */
void Nidec24H::isr0() {
    if (instance0 != nullptr) {
        instance0->updateSpeed();
    }
}

/**
 * @brief Handle interrupt 1. Forward it to stored instance 1.
 */
void Nidec24H::isr1() {
    if (instance1 != nullptr) {
        instance1->updateSpeed();
    }
}

/**
 * @brief Handle interrupt 2. Forward it to stored instance 2.
 */
void Nidec24H::isr2() {
    if (instance2 != nullptr) {
        instance2->updateSpeed();
    }
}

/**
 * @brief Handle interrupt 3. Forward it to stored instance 3.
 */
void Nidec24H::isr3() {
    if (instance3 != nullptr) {
        instance3->updateSpeed();
    }
}

/* 
 * Class for Nidec 24H brushless motor.
 * This class provides functions for interfacing with the motor.
 * 
 * init() - initialize the motor class.
 * setPower(int power) - set the power level for the motor
 * brake() - set the motor to brake.
 * setDirection(Direction invertDir) - set the default the motor direction.
 * getDirection() - get the motor direction
 */

#include "Nidec24H.h"

/**
 * @brief Constructor for the class.
 * 
 * @param pwmPin - pin to control speed using PWM.
 * @param dirPin - pin to toggle motor direction.
 * @param brakePin - pin to brake to toggle motor braking.
 */
Nidec24H::Nidec24H(int pwmPin, int dirPin, int brakePin) 
    : pwmPin(pwmPin), dirPin(dirPin), brakePin(brakePin) {
        
}

/**
 * @brief Initialize the motor.
 */
void Nidec24H::init() {
    //Setup Pins
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(brakePin, OUTPUT);

    //Set some initial values
    digitalWrite(dirPin, LOW);
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


void Nidec24H::brake() {
    //set to some power
	analogWrite(pwmPin, 50);

    //Turn on the brake
    digitalWrite(brakePin, LOW);
}
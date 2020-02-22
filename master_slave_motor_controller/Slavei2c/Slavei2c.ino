/* 1/15/2020
 * Christian Weaver
 * 
 * This code offloads the encoder and motor control management froma "master" Arduino to a "slave" 
 * Arduino. The "master" arduino sends an RPM value (of type int) over I2C to a "slave" Arduino
 * that is connected to an encoder and motor controller. The slave Arduino will regulate the motor
 * and keep it running at the RPM sent by the master Arduino. Multiple "slave" Arduinos may be 
 * connected to the same "master" arduion. 
 * 
 * Wiring:
 * _______________           _______________
 * | Arduino  GND|---------->|GND Arduino  |
 * | (slave)     |           |   (master)  |
 * |             |           |             |
 * |          SDA|---------->|SDA          |
 * |             |           |             |
 * |             |           |             |
 * |          SCL|---------->|SCL          |
 * |             |           |             |
 * |             |           |             |
 * ---------------           ---------------
 */

#include <Wire.h> //handels I2C
#include "BTS7960.h" //motor controller library
#include "AS5134.h" //motor encoder library
#include "SpeedController.h" //PID library

#define INIT_POWER 20
#define MAX_SPEED 30
#define DATA_PIN 4
#define CS_PIN 3
#define CLK_PIN 2


//Motor 1
const int REN1 = 7;
const int LEN1 = 8;
const int PWM1 = 6;


//instantiate a structure for initializing the SpeedController object
//InitSpeedControllerValues setupValues; //the structure already has default values set

//***Motor 1***
//Create the motor
BTS7960 motor(REN1, LEN1, PWM1, true);
SpeedController motorSpeedController(new AS5134(DATA_PIN, CS_PIN, CLK_PIN));


int mySpeed = 0; //this global variable is used to set the desired speed


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  int c = Wire.read();
  mySpeed = c;    // set the lower byte
  c = Wire.read();
  mySpeed |= c<<8; // set the upper byte
}




//=====setup==============================
void setup() {
 
  //start I2C
  Wire.begin(1);                // join i2c bus with address #1
  Wire.onReceive(receiveEvent); // register event

  //Initialize the motor
  motor.init();
  motor.stop();
  motor.run(INIT_POWER);
  
  //Start serial
  Serial.begin(115200);  
}


//=====loop==============================
void loop() {

//  static SpeedController motorSpeedController(SpeedControllerInitialValues, new AS5134(42, 52, 31));
  
  static int motorPower = INIT_POWER;
  static int motorSpeed;
  static int lastPrint = millis();
  
  motorPower = motorSpeedController.motorSpeedToPower(mySpeed); // checks the encoder and sets the power to keep the motor at the desired speed
  motorSpeed = motorSpeedController.getSpeed1(); // gets the speed from the encoder
 

  motor.run(motorPower);

  Serial.println(mySpeed);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print("Power: ");
    Serial.print(motorPower);
    Serial.print(", Speed: ");
    Serial.println(motorSpeed);
    lastPrint = millis();
  }
}

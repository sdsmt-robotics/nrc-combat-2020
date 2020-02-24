#include "Nidec24H.h"

#define INIT_MTR_POWER 30

//pin
int fs_pin = 2;

Nidec24H motor(11, 7, 8);

//Speed Control
int rpm = 0;
long fs_time = 0;


/**
 * Print the state of the motor controller.
 */
void printControllerState(bool motorRunning, int motorPower) 
{
  if (motorRunning) {
    Serial.print("RUNNING @ p: ");
    Serial.print(motorPower);
    Serial.print(", s: ");
    Serial.println(rpm);
  } else {
    Serial.println("STOPPED");
  }
}


/**
 * @brief Setup code
 */
void setup() {
  //Setup serial
  Serial.begin(115200);
  
  //Add the pin counter
  pinMode(fs_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(fs_pin), counter, RISING);

  motor.init();
  motor.setDirection(REVERSE);
  motor.setPower(INIT_MTR_POWER);
}

/**
 * @brief Main loop
 */
void loop() {
  static bool motorRunning = true;
  static int motorPower = INIT_MTR_POWER;
  
  //Read user input if available
  if(Serial.available()){
    //Read and then kill the rest
    char readVal = Serial.read();
    while(Serial.available()){Serial.read();}

    switch (readVal) {
      case ' ':
        //toggle the motor on or off
        if (motorRunning) {
          motor.brake();
          motorRunning = false;
        } else {
          motor.setPower(motorPower);
          motorRunning = true;
        }
        break;
      case '+':
      case '=':
        motorPower += 10;
        motorRunning = true;
        motor.setPower(motorPower);
        break;
      case '-':
      case '_':
        motorPower -= 10;
        motorRunning = true;
        motor.setPower(motorPower);
        break;
      default:
        break;
    }
  }

  //Print the current state
  printControllerState(motorRunning, motorPower);

  delay(100);


  //Sweep through the different functions
  /*
  Serial.println("Running...");
  motor.setPower(30);
  delay(3000);

  Serial.println("Braking...");
  motor.brake();
  delay(3000);
  
  Serial.println("Running...");
  motor.setPower(30);
  delay(3000);

  Serial.println("Stopping...");
  motor.setPower(0);
  delay(3000);
  */
}

/**
 * @brief Counter to calculate rpm from the frequency pulses
 * 
 */
void counter() {
  if (fs_time == 0) {
    fs_time = micros();
  } else {
    rpm = 10000000.0/(micros()-fs_time);
    fs_time = micros();
  }
}

#include "Nidec24H.h"
#include "Filter.h"
#include "SpeedController.h"

#define INIT_POWER 30
#define INIT_TARGET 1000

//motor we are running
Nidec24H motor(11, 7, 8, 2);

//motor speed controller
SpeedController speedControl;


//=====SETUP=========================================================
/**
   @brief Setup code
*/
void setup() {
  //Setup serial
  Serial.begin(115200);

  speedControl.setPIDConsts(0.08, 1, 0.02);
  speedControl.setOutputLimits(5, 255);
  speedControl.setTarget(INIT_TARGET);

  //initialize the motor
  motor.init();
  motor.setDirection(REVERSE);
  motor.setPower(INIT_POWER);
}


//=====LOOP=========================================================
/**
   @brief Main loop
*/
void loop() {

  //****Uncomment one of the below tests to see how it works!****//

  //Test basic motor functions (run at power, set to zero power, brake.)
  //testMotorFunctions();

  //Run at a set power and output the raw speed vals vs the filtered ones.
  //testFilter();

  //Run the motor at some constant speed using PID control
  //testSetSpeed();

  //Change the motor speed every three seconds
  testChangingSpeed();

  //control motor speed using the serial console
  //testSpeedControls();
}

//=====TEST FUNCTIONS================================================

/**
 * Test basic motor functionality.
 */
void testMotorFunctions() {
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
}

/**
 * Run the motor at some constant power level and observe the raw speed vs the filtered speed.
 */
void testFilter() {
  const int power = 128; //power level
  static long lastPrint = millis();

  //Do the calculation just so we get the same amount of lag as normal
  speedControl.calcOutput(motor.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(motor.getFilteredSpeed());
    Serial.print(",\t");
    Serial.println(motor.getSpeed());
    
    lastPrint = millis();
  }
}

/**
 * Run the motor at a set speed using the PID control.
 */
void testSetSpeed() {
  static int power = 0; //power level
  static long lastPrint = millis();

  //set the target speed
  speedControl.setTarget(1000);

  //Get the power level from the speed controller
  power = speedControl.calcOutput(motor.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(speedControl.getTarget());
    Serial.print(",\t");
    Serial.println(motor.getFilteredSpeed());
    
    lastPrint = millis();
  }
}


/**
 * Test changing the speed to a new value every three seconds.
 */
void testChangingSpeed() {
  const int speeds[] = {1000, 600, 2000};
  const int numSpeeds = 3;
  static int speedIndex = 0;
  static long lastSpeedChange = millis();
  
  static int power = 0; //power level
  
  static long lastPrint = millis();

  //go to the next speed if past time
  if (millis() - lastSpeedChange > 3000) {
    speedIndex++;
    lastSpeedChange = millis();
    if (speedIndex >= numSpeeds) {
      speedIndex = 0;
    }
  }

  //set the target speed
  speedControl.setTarget(speeds[speedIndex]);

  //Get the power level from the speed controller
  power = speedControl.calcOutput(motor.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(speedControl.getTarget());
    Serial.print(",\t");
    Serial.println(motor.getFilteredSpeed());
    
    lastPrint = millis();
  }
}

/**
 * Control motor using commands sent from the serial console.
 * 
 * Space: toggle motor on/off
 * number: set the motor to that speed
 */
void testSpeedControls() {
  static bool motorRunning = true;
  static int power = 0; //power level
  
  static long lastPrint = millis();

  
  //Read user input if available
  if (Serial.available()) {
    //If space, toggle motor
    if (Serial.peek() == ' ') {
      motorRunning = !motorRunning;
      while (Serial.available()) {
        Serial.read();
      }
    } else {
      //read the new speed value
      motorRunning = true;
      speedControl.setTarget(Serial.parseInt());
    }
  }

  
  //update the motor
  if (motorRunning) {
    power = speedControl.calcOutput(motor.getFilteredSpeed());
    motor.setPower(power);
  } else {
    motor.setPower(0);
  }
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(speedControl.getTarget());
    Serial.print(",\t");
    Serial.println(motor.getFilteredSpeed());
    
    lastPrint = millis();
  }
}

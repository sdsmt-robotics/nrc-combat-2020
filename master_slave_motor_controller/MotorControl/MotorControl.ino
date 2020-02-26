#include "Nidec24H.h"
#include "Filter.h"

#define INIT_POWER 30
#define INIT_TARGET 1000


Nidec24H motor(11, 7, 8, 2);



//=====SPEED CONTROL=========================================================
//Reading speed
int target = INIT_TARGET;

float integral = 0;
float derivative = 0;
int error = 0;

Filter powerFilter(10);

//Define constants for the control
const double k = 0.3;
const int minPower = 30;
const int maxPower = 40;
const int proportionalThresh = 1080;
const int lowSpeedThresh = 360;
const int stopThresh = 25;


int computeOutput(int curRpm, bool reset = false) {
  //Define some constants
  const int maxOutput = 255;    //max output value
  const int minOutput = 0;   //min output value
  const int maxOutCap = 255;     //max output we will send
  const int minOutCap = 5;    //min output we will send
  //const int maxInput  = 5000;  //max possible input value
  //const int minInput  = 5; //min possible input value
  static long lastTime = 0;
  static int lastError = 0;
  long curTime = micros();

  //Define PID constants
  const float kp = 0.08;
  const float ki = 1;
  const float kd = 0.02;


  int output = 0;
  float elapsedTime;


  // Skip if first loop so we can get initial values
  if (lastTime == 0 || reset) {
    lastTime = micros();
    lastError = target - curRpm;
    integral = 0;
    return (maxOutput + minOutput) / 2;
  }

  //update the time
  elapsedTime = (curTime - lastTime) / 1000000.0;

  lastTime = curTime;

  //get the error
  error = target - curRpm;

  //Get the integral of the error
  integral += (float)error * elapsedTime;

  //Get the derivative of the error
  derivative = (float)(error - lastError) / elapsedTime;

  //calculate the output
  output = kp * error + ki * integral + kd * derivative;

  //update for next call
  lastError = error;

  //cap the output and return
  return constrain(output, minOutCap, maxOutCap);
}


//=====SETUP=========================================================
/**
   @brief Setup code
*/
void setup() {
  //Setup serial
  Serial.begin(115200);

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
  powerFilter.filter(computeOutput(motor.getFilteredSpeed()));

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
  target = 1000;

  //Get the power level from the speed controller
  power = powerFilter.filter(computeOutput(motor.getFilteredSpeed()));

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(target);
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
  target = speeds[speedIndex];

  //Get the power level from the speed controller
  power = powerFilter.filter(computeOutput(motor.getFilteredSpeed()));

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(target);
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
      target = Serial.parseInt();
      computeOutput(motor.getFilteredSpeed(), true);
    }
  }

  
  //update the motor
  if (motorRunning) {
    power = powerFilter.filter(computeOutput(motor.getFilteredSpeed()));
    motor.setPower(power);
  } else {
    motor.setPower(0);
  }
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(target);
    Serial.print(",\t");
    Serial.println(motor.getFilteredSpeed());
    
    lastPrint = millis();
  }
}

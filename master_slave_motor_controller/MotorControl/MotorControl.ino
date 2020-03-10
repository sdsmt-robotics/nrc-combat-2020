/*
 * Testing program for the Nidec brushless motor speed controller class.
 * This is set up to run with: 
 *   - a Nidec brushless motor 
 *   - CUI encoder set to 48 ticks per rotation
 *   - Arduino Uno.
 * 
 * This will also run on an ATtiny.
 * 
 */

#include "Nidec24H.h"
#include "Filter.h"
#include "SpeedController.h"


#define INIT_POWER 30
#define INIT_TARGET 600
#define LED_PIN 7
#define KNOB_PIN A5

//10 - pwmPin
//7 - dirPin
//8 - brakePin
//2 - fgPin (interrupt 0)
//Nidec24H motor(6, 3, 0, 8);   //ATTiny Pins
Nidec24H motor(11, 7, 8, 2);   //Arduino Uno Pins

//motor speed controller
SpeedController speedControl;



//=====SETUP=========================================================
/**
   @brief Setup code
*/
void setup() {
  //Setup serial
  Serial.begin(38400);
  
  pinMode(LED_PIN, OUTPUT);

  //initialize the speed control
  //kp - amplitude
  //kd - damper
  //ki - constant error correction
  speedControl.setPIDConsts(0.08, 1.4, 0.003);
  speedControl.setOutputLimits(10, 255);
  speedControl.setTarget(INIT_TARGET);
  
  //initialize the motor
  motor.init();
  motor.setDirection(REVERSE);
  motor.setPower(INIT_POWER);

  
  /*
   * WGM10, WGM12: Fast PWM, 8-bit TOP=255
   * CS10: No prescaler
   * COM1A1: Pin 6 to LOW on match with OCR1A and to HIGH on TOP
   */
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  pinMode(KNOB_PIN, INPUT);
}


//=====LOOP=========================================================
/**
   @brief Main loop
*/
void loop() {

  //****Uncomment one of the below tests to see how it works!****//

  //Test basic motor functions (run at power, set to zero power, brake.)
  //testMotorFunctions();

  //test controlling motor power using a knob
  //testKnobPower();

  //Run the motor at some constant power level and read the tick frequency.
  //testTickFreq();

  //Run at a set power and output the raw speed vals vs the filtered ones.
  //testFilter();

  //Run the motor at some constant speed using PID control
  //testSetSpeed();

  //Control motor speed target using a knob on A5.z
  //testKnobControl();

  //Test setting motor speed to an oscilating curve
  //testOscSpeed();

  //Change the motor speed every three seconds
  //testChangingSpeed();

  testDrivingSpeed();

  //control motor speed using the serial console
  //testSpeedControls();
  /*Serial.println("on");
  digitalWrite(LED_PIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  Serial.println("off");
  digitalWrite(LED_PIN, HIGH);    // turn the LED off by making the voltage LOW
  delay(1000);    */                   // wait for a second
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
    motor.setPower(80);
    delay(3000);

    Serial.println("Stopping...");
    motor.setPower(0);
    delay(3000);
}


/**
 * Control motor speed target using a knob on A5.
 */
void testKnobPower() {
  static int power = 0; //power level
  static long lastPrint = millis();

  //set the power
  power = analogRead(KNOB_PIN) >> 2;

  //For timing consistancy
  //speedControl.calcOutput(motor.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    //Serial.print(speedControl.getTarget());
    Serial.println(motor.lastInterval);
    //Serial.print(",\t");
    //Serial.println(motor.getSpeed());
    
    lastPrint = millis();
  }
}

/**
 * Run the motor at some constant power level and read the tick frequency.
 */
void testTickFreq() {
  const int power = 100; //power level
  static long lastPrint = millis();

  //Do the calculation just so we get the same amount of lag as normal
  //speedControl.calcOutput(motor.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);

  //motor.ticks = 0;
  delay(100);
  Serial.println(motor.ticks);
}

/**
 * Run the motor at some constant power level and observe the raw speed vs the filtered speed.
 */
void testFilter() {
  const int power = 125; //power level
  static long lastPrint = millis();
  static int speedKalman;

  //Do the calculation just so we get the same amount of lag as normal
  speedControl.calcOutput(motor.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(motor.getSpeed());
    Serial.print(",\t");
    Serial.println(motor.getFilteredSpeed());
    
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
  speedControl.setTarget(2000);

  //Get the power level from the speed controller
  power = speedControl.calcOutput(motor.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    //Serial.print(speedControl.getTarget());
    //Serial.print(",\t");
    //Serial.println(motor.getFilteredSpeed());
    Serial.print(speedControl.integral);
    Serial.print(",\t");
    Serial.println(speedControl.lastError);
    
    lastPrint = millis();
  }
}


/**
 * Control motor speed target using a knob on A5.
 */
void testKnobControl() {
  static int power = 0; //power level
  static long lastPrint = millis();

  //set the target speed
  speedControl.setTarget(analogRead(KNOB_PIN) << 2);

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
 * Run the motor at a set speed using the PID control.
 */
void testOscSpeed() {
  const int rotation = 30;
  const float c = rotation * 2 * M_PI / 60000000.0;
  
  static int power = 0; //power level
  
  static long lastPrint = millis();
  
  int driveSpeed = 1000 + 1000 * cos(micros() * c);

  //set the target speed
  speedControl.setTarget(driveSpeed);

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
  
    motor.ticks = 0;
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
    //Serial.print(",\t");
    //Serial.print(motor.getSpeed());
    Serial.print(",\t");
    Serial.println(motor.getFilteredSpeed());
    
    lastPrint = millis();
  }
}


/**
 * Test changing the speed to a new value every three seconds and display PID values.
 */
void testShowPID() {
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
    Serial.print(speedControl.lastError);
    Serial.print(",\t");
    Serial.print(speedControl.integral);
    Serial.print(",\t");
    Serial.println(speedControl.derivative);
    
    lastPrint = millis();
  }
}

/**
 * Test changing the speed to a new value every three seconds.
 */
void testDrivingSpeed() {
  const int numSpeeds = 3;
  static int speedIndex = 0;
  static long lastSpeedChange = millis();
  const int rotation = 350; //rpm
  const int translation = 3;  //feet per sec
  
  const float c = rotation * 2 * M_PI / 60000000.0;
  const int rotateForTranslate = translation * 97.021;  //amount of rotation needed to get the translation speed
  
  static int power = 0; //power level
  
  static long lastPrint = millis();
  
  int driveSpeed = 1400 + rotateForTranslate * cos(micros() * c);

  //set the target speed
  speedControl.setTarget(driveSpeed);

  //Get the power level from the speed controller
  power = speedControl.calcOutput(motor.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 10) {
    Serial.print(driveSpeed);
    Serial.print(",\t");
    Serial.print(motor.getFilteredSpeed());
    Serial.print(",\t");
    Serial.println(0);
    
    lastPrint = millis();
  }
  
    motor.ticks = 0;
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

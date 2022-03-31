#include <Arduino.h>
#include <math.h>

#include "accelerometer.h"
#include "controller.h"
#include "encoder.h"
#include "imu.h"
#include "led.h"
#include "motor.h"

// ---- PINS ----

#define STATUS_LED_PIN 5
#define ESP_STATUS_LED_PIN 2 // internal led on esp32 (its the blue one)

#define LED_STRIP_1 26
#define LED_STRIP_2 27
#define LED_STRIP_3 14

#define ENC_1A 36
#define ENC_1B 39
#define ENC_2A 34
#define ENC_2B 35
#define ENC_3A 12
#define ENC_3B 13

#define ADC_SELECT 23
#define ADC_RX 19
#define ADC_TX 18

#define MOTOR_1 32
#define MOTOR_1_RMT_CHANNEL 3
#define MOTOR_2 33
#define MOTOR_2_RMT_CHANNEL 4
#define MOTOR_3 25
#define MOTOR_3_RMT_CHANNEL 5

// ---- MISC ----

#define ADAFRUIT_RMT_CHANNEL_MAX 3 // LED strips can use 3 RMT channels 0 - 2

// ---- SENSOR STUFF ----

IMU imu;

// Encoder e1(ENC_1A, ENC_1B, 360);
// Encoder e2(ENC_2A, ENC_2B, 360);
// Encoder e3(ENC_3A, ENC_3B, 360);

Accelerometers accel(Serial1, ADC_SELECT, ADC_TX,
                     ADC_RX); // note: tx and rx are fliped

// ---- LED STUFF ----

LedStrip strip1(LED_STRIP_1);
LedStrip strip2(LED_STRIP_2);
LedStrip strip3(LED_STRIP_3);

// ---- MOTOR STUFF ----

Motor m1(MOTOR_1, MOTOR_1_RMT_CHANNEL);
Motor m2(MOTOR_2, MOTOR_2_RMT_CHANNEL);
Motor m3(MOTOR_3, MOTOR_3_RMT_CHANNEL);

const int SPIN_MAX_POWER = 300; // max motor power when enabled
const int SPIN_MIN_POWER = 100; // min motor power when enabled

// max movement power. this is the magnitude of max power when moving
const int TRANS_MAX_POWER = 50;

const long MOTOR_INIT_TIME = 4000; // wait time for motor init in miliseconds

const long MOTOR_UPDATE_INTERVAL = 10; // motor update loop time in microseconds
long last_motor_update = micros();

// ---- LOOP STUFF ----

bool robot_enabled = false;

const long LOOP_INTERVAL = 1000; // microsec
long last_loop = micros();

// ---- CONTROLLER STUFF ----

Controller controller(Serial2);

// movement
// range -1 to 1, negative is reverse
float x = 0;
float y = 0;

// spin speed
//  range -1 to 1, negative is reverse
float spin = 0;

bool flip = false;

bool full_send = false;

// ---- SENSOR FUNCTIONS ----

/**
   @brief Initializes all sensors.
*/
void initSensors() {
  // init IMU
  if (!imu.init()) {
    Serial.println("\nimu fail init");
  }

  // init accelerometers
  accel.init();

  // init encoders
  //  e1.init();
  //  e2.init();
  //  e3.init();
}

/**
   @brief Updates all sensors.
*/
void updateSensors() {
  // update IMU
  imu.update();

  // update accelerometers
  accel.update();
}

// ---- MOTOR FUNCTIONS ----

/**
   @brief Initializes all motors.
*/
void initMotors() {
  m1.init();
  m2.init();
  m3.init();
}

/**
   @brief Sends a DShot command to all motors.
   @param cmd command
*/
void sendAllMotorCommand(int cmd) {
  m1.send_dshot_command(cmd);
  m2.send_dshot_command(cmd);
  m3.send_dshot_command(cmd);
}

/**
   @brief Sets a power for all motors.
   @param power power value to set. -1000 to 1000
*/
void sendAllMotorPower(int power) {
  m1.set_speed_signed(power);
  m2.set_speed_signed(power);
  m3.set_speed_signed(power);
}

/**
   @brief Arms all motors by initialy sending the 0 command and then sets the
   power to 0 for the initialization. This blocks for MOTOR_INIT_TIME
   miliseconds.
*/
void armMotors() {
  long init_motor_start = millis();
  sendAllMotorCommand(0);
  while (millis() - init_motor_start < MOTOR_INIT_TIME) {
    sendAllMotorPower(0);
  }
}

/**
   @brief Updates drive motor powers with control and angle inputs.
   @param spin Spin speed modifier. -1.0 to 1.0
   @param x X direction movement modifier. -1.0 to 1.0
   @param y Y direction movement modifier. -1.0 to 1.0
   @param angle Current angle of robot in radians
*/
void updateDrive(float spin, float x, float y, float angle) {

  // constrain inputs to prevent problems with math
  spin = constrain(spin, -1.0, 1.0);
  x = constrain(x, -1.0, 1.0);
  y = constrain(y, -1.0, 1.0);

  // define variables and constants for calculation
  int power[3] = {0, 0, 0};

  const float PI2_OVER_3 = 2 * PI / 3;

  const float m1_offset = 0.0;
  const float m2_offset = PI2_OVER_3;
  const float m3_offset = -PI2_OVER_3;

  float phase = PI2_OVER_3;

  float spin_power = 0;

  float trans_power = TRANS_MAX_POWER * sqrtf((x * x) + (y * y));
  float trans_phase = atan2f(y, x);

  float comp_angle = angle + trans_phase;

  if (spin > 0) {

    // robot upright

    comp_angle += phase;

    spin_power = SPIN_MIN_POWER + ((SPIN_MAX_POWER - SPIN_MIN_POWER) * spin);

    power[0] = (spin_power + (trans_power * sinf(comp_angle + m1_offset)));
    power[1] = (spin_power + (trans_power * sinf(comp_angle + m2_offset)));
    power[2] = (spin_power + (trans_power * sinf(comp_angle + m3_offset)));

  } else if (spin < 0) {

    // robot fliped over

    comp_angle -= phase;

    spin_power = -SPIN_MIN_POWER + ((SPIN_MAX_POWER - SPIN_MIN_POWER) * spin);

    power[0] = (spin_power - (trans_power * sinf(comp_angle + m1_offset)));
    power[1] = (spin_power - (trans_power * sinf(comp_angle + m2_offset)));
    power[2] = (spin_power - (trans_power * sinf(comp_angle + m3_offset)));
  }

  // // debug
  // if (!controller.button(DOWN)) {
  //   Serial.print(power[0]);
  //   Serial.print("\t");
  //   Serial.print(power[1]);
  //   Serial.print("\t");
  //   Serial.print(power[2]);
  //   Serial.print("\t");
  //   Serial.println();
  // }
  // // end debug

  // update motor power
  m1.set_speed_signed(power[0]);
  m2.set_speed_signed(power[1]);
  m3.set_speed_signed(power[2]);
}

// ---- LED FUNCTIONS ----

/**
   @brief Turns both status LEDs on and off
   @param on true = on, false = off
*/
void setStatusLED(bool on) {
  if (on) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    digitalWrite(ESP_STATUS_LED_PIN, HIGH);
  } else {
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(ESP_STATUS_LED_PIN, LOW);
  }
}

/**
   @brief Initializes all LED strips.
*/
void initLEDStrips() {
  strip1.init();
  strip2.init();
  strip3.init();
}

/**
 * @brief Sets up and initializes LED strips and status LED
 */
void initLEDs() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ESP_STATUS_LED_PIN, OUTPUT);

  setStatusLED(false);

  initLEDStrips();
}

/**
   @brief Updates all LED strips to set colors.
*/
void showLEDStrips() {
  strip1.show();
  strip2.show();
  strip3.show();
}

/**
   @brief Sets all LED strips to RGB color.
   @param red red 0 to 255
   @param green red 0 to 255
   @param blue red 0 to 255
*/
void fillLEDs(uint8_t red, uint8_t green, uint8_t blue) {
  strip1.fillColor(strip1.makeColor(red, green, blue));
  strip2.fillColor(strip2.makeColor(red, green, blue));
  strip3.fillColor(strip3.makeColor(red, green, blue));
}

// ---- MAIN FUNCTIONS ----

void setup() {

  Serial.begin(115200);

  initLEDs();

  // set LED strips to red
  fillLEDs(255, 0, 0);
  showLEDStrips();

  controller.init();

  initSensors();

  initMotors();

  // set LED strips to yellow
  fillLEDs(255, 255, 0);
  showLEDStrips();

  armMotors();

  // set LED strips to green
  fillLEDs(0, 255, 0);
  showLEDStrips();
}

void loop() {
  // main loop timer
  if (micros() - last_loop > LOOP_INTERVAL) {
    last_loop = micros();

    // update controls
    if (controller.connected()) {

      // turn status on and set LED strips to green
      setStatusLED(true);
      fillLEDs(0, 255, 0);

      if (controller.buttonClick(RIGHT)) {
        robot_enabled = true;
        spin = 0.1;
        if (flip)
          spin = -0.1;
        Serial.println("run");
      }
      if (controller.buttonClick(LEFT)) {
        spin = 0;
        robot_enabled = false;
        Serial.println("stop");
      }

      x = controller.joystick(RIGHT, X);
      y = controller.joystick(RIGHT, Y);

      if (controller.dpadClick(UP) && robot_enabled) {
        if (flip) {
          spin -= 0.1;
          if (spin < -1.0) {
            spin = -1.0;
          }
        } else {
          spin += 0.1;
          if (spin > 1.0) {
            spin = 1.0;
          }
        }
        Serial.println("spin up");
        Serial.println(spin);
      }

      if (controller.dpadClick(DOWN) && robot_enabled) {
        if (flip) {
          spin += 0.1;
          if (spin > -0.1) {
            spin = -0.1;
          }
        } else {
          spin -= 0.1;
          if (spin < 0.1) {
            spin = 0.1;
          }
        }
        Serial.println("spin down");
        Serial.println(spin);
      }

      if (controller.dpadClick(LEFT) && !robot_enabled) {
        flip = false;
        Serial.println("spin normal");
      }

      if (controller.dpadClick(RIGHT) && !robot_enabled) {
        flip = true;
        Serial.println("spin reverse");
      }

      if (controller.button(DOWN)) {
        Serial.println((imu.getAngle() * RAD_2_DEG));
        Serial.println();
      }

      if (controller.button(UP)) {
        Serial.println(accel.getVelocity());
        Serial.println();
      }

      if (controller.bumper(RIGHT) && robot_enabled) {
        full_send = true;
        Serial.println("FULL SEND!!!");
      } else {
        full_send = false;
      }

    } else {

      // controller disconnected

      // set status to off and all LED strips to red
      setStatusLED(false);
      fillLEDs(255, 0, 0);

      // disable robot
      robot_enabled = false;
      Serial.println("no remote");
    }

    updateSensors();
  }

  // motor update timer
  if (micros() - last_motor_update > MOTOR_UPDATE_INTERVAL) {
    last_motor_update = micros();

    if (robot_enabled) {

      // run motors
      if (!full_send) {
        updateDrive(spin, x, y, imu.getAngle());

        // set led strips to purple
        fillLEDs(255, 0, 255);

      } else {
        // full send mode
        if (flip) {
          sendAllMotorPower(-1000);
        } else {
          sendAllMotorPower(1000);
        }
        // set led strips to red
        fillLEDs(255, 0, 0);
      }

    } else {
      // robot disabled. stop all motors
      sendAllMotorPower(0);
    }
    showLEDStrips();
  }
}

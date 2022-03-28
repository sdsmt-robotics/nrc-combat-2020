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

#define LED_STRIP_1 26
#define LED_STRIP_2 27
#define LED_STRIP_3 25

#define ENC_1A 14
#define ENC_1B 12
#define ENC_2A 4
#define ENC_2B 0
#define ENC_3A 2
#define ENC_3B 15

#define ADC_SELECT 13

#define MOTOR_1 32
#define MOTOR_1_RMT_CHANNEL 0
#define MOTOR_2 33
#define MOTOR_2_RMT_CHANNEL 1
#define MOTOR_3 25
#define MOTOR_3_RMT_CHANNEL 2

// ---- SENSOR STUFF ----

IMU imu;

Encoder e1(ENC_1A, ENC_1B, 360);
Encoder e2(ENC_2A, ENC_2B, 360);
Encoder e3(ENC_3A, ENC_3B, 360);

ADC acc1(Serial1);
ADC acc2(Serial1);

// Accelerometers accel(Serial2, ADC_SELECT);

// ---- LED STUFF ----

LedStrip strip1(LED_STRIP_1);
LedStrip strip2(LED_STRIP_2);
LedStrip strip3(LED_STRIP_3);

// ---- MOTOR STUFF ----

Motor m1(MOTOR_1, MOTOR_1_RMT_CHANNEL);
Motor m2(MOTOR_2, MOTOR_2_RMT_CHANNEL);
Motor m3(MOTOR_3, MOTOR_3_RMT_CHANNEL);

const int SPIN_MAX_POWER = 300;
const int SPIN_MIN_POWER = 100;
const int TRANS_MAX_POWER = 50;

const long MOTOR_INIT_TIME = 4000; // milisec

const long MOTOR_UPDATE_INTERVAL = 10;
long last_motor_update = micros();

// ---- LOOP STUFF ----

bool run = false;

const long LOOP_INTERVAL = 10; // microsec
long last_loop = micros();

// ---- CONTROLLER STUFF ----

Controller controller(Serial2);

float x = 0;
float y = 0;
float spin = 0;

bool flip = false;

// ---- SENSOR FUNCTIONS ----

void initAccelerometers() {
  pinMode(ADC_SELECT, OUTPUT);
  digitalWrite(ADC_SELECT, LOW);
  acc1.init();
  digitalWrite(ADC_SELECT, HIGH);
  acc2.init();
}

void initEncoders() {
  e1.init();
  e2.init();
  e3.init();
}

void initIMU() { imu.init(); }

void initSensors() {
  // initAccelerometers();
  initIMU();
}

float readAccelerometers() {
  float data[2] = {0};
  digitalWrite(ADC_SELECT, LOW);
  data[0] = acc1.readData();
  digitalWrite(ADC_SELECT, HIGH);
  data[1] = acc2.readData();
  return data[1];
}

// ---- MOTOR FUNCTIONS ----

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

  spin = constrain(spin, -1.0, 1.0);
  x = constrain(x, -1.0, 1.0);
  y = constrain(y, -1.0, 1.0);

  int power[3] = {0, 0, 0};

  const float PI2_OVER_3 = 2 * PI / 3;

  const float m1_offset = 0.0;
  const float m2_offset = PI2_OVER_3;
  const float m3_offset = -PI2_OVER_3;

  float phase = PI2_OVER_3;

  float spin_power = 0;

  float trans_power = TRANS_MAX_POWER * sqrtf((x * x) + (y * y));
  float trans_phase = atan2f(y, x);

  float comp_angle = angle + trans_phase + phase;

  if (spin > 0) {

    spin_power = (SPIN_MAX_POWER * spin) + SPIN_MIN_POWER;

    power[0] = (spin_power + (trans_power * sinf(comp_angle + m1_offset)));
    power[1] = (spin_power + (trans_power * sinf(comp_angle + m2_offset)));
    power[2] = (spin_power + (trans_power * sinf(comp_angle + m3_offset)));

  } else if (spin < 0) {

    spin_power = (SPIN_MAX_POWER * spin) - SPIN_MIN_POWER;

    power[0] = (spin_power - (trans_power * sinf(comp_angle + m1_offset)));
    power[1] = (spin_power - (trans_power * sinf(comp_angle + m2_offset)));
    power[2] = (spin_power - (trans_power * sinf(comp_angle + m3_offset)));
  }

  m1.set_speed_signed(power[0]);
  m2.set_speed_signed(power[1]);
  m3.set_speed_signed(power[2]);
}

// ---- LED FUNCTIONS ----

/**
 * @brief Sets up and initializes LED strips and status LED
 */
void initLEDs() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  strip1.fillColor(strip1.color(255, 255, 0));
  strip2.fillColor(strip2.color(255, 255, 0));
  strip3.fillColor(strip3.color(255, 255, 0));
  strip1.show();
  strip2.show();
  strip3.show();

  digitalWrite(STATUS_LED_PIN, HIGH);
}

void showLEDs() {
  strip1.show();
  strip2.show();
  strip3.show();
}

void fillLEDs(uint8_t r, uint8_t g, uint8_t b) {
  strip1.fillColor(strip1.color(r, g, b));
  strip1.fillColor(strip1.color(r, g, b));
  strip1.fillColor(strip1.color(r, g, b));
}

// ---- MAIN FUNCTIONS ----

void setup() {

  Serial.begin(115200);

  controller.init();

  initLEDs();

  initMotors();

  initSensors();

  armMotors();
}

void loop() {
  if (micros() - last_loop > LOOP_INTERVAL) {
    last_loop = micros();
    if (controller.connected()) {

      digitalWrite(STATUS_LED_PIN, HIGH);

      if (controller.buttonClick(RIGHT)) {
        run = true;
        spin = 0.0;
        Serial.println("run");
      }
      if (controller.buttonClick(LEFT)) {
        spin = 0;
        run = false;
        Serial.println("stop");
      }

      x = controller.joystick(RIGHT, X);
      y = controller.joystick(RIGHT, Y);

      if (controller.dpadClick(UP) && run) {
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
      }

      if (controller.dpadClick(DOWN) && run) {
        if (flip) {
          spin += 0.1;
          if (spin > 0.0) {
            spin = 0.0;
          }
        } else {
          spin -= 0.1;
          if (spin < 0.0) {
            spin = -1.0;
          }
        }
        Serial.println("spin down");
      }

      if (controller.dpadClick(LEFT) && !run) {
        flip = false;
        Serial.println("spin normal");
      }

      if (controller.dpadClick(RIGHT) && !run) {
        flip = true;
        Serial.println("spin reverse");
      }

    } else {

      digitalWrite(STATUS_LED_PIN, LOW);

      run = false;
      Serial.println("no remote");
      Serial.println(readAccelerometers());
    }

    imu.update();
  }
  if (micros() - last_motor_update > MOTOR_UPDATE_INTERVAL) {
    last_motor_update = micros();
    if (run) {
      updateDrive(spin, x, y, imu.getAngle());
    } else {
      sendAllMotorPower(0);
    }
  }
}

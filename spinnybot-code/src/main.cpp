#include <Arduino.h>
#include <math.h>

// #include "accelerometer.h"
#include "constants.h"
#include "controller.h"
#include "driveFuncs.h"
#include "encoder.h"
#include "imu.h"
#include "led.h"
#include "motor.h"
#include "ota.h"
#include "pins.h"
#include "utility.h"

#define AP_SSID "SPINNY" // ssid for robot

// ---- SENSOR STUFF ----

IMU imu;

Encoder e1(ENC_1, 360);
Encoder e2(ENC_2, 360);
Encoder e3(ENC_3, 360);

/* Accelerometers accel(Serial1, ADC_SELECT, ADC_TX,
                     ADC_RX); // note: tx and rx are fliped
                     */

// ---- LED STUFF ----

LedStrip strip1(LED_STRIP_1);
LedStrip strip2(LED_STRIP_2);
LedStrip strip3(LED_STRIP_3);
LedStrip* strips[] = {&strip1, &strip2, &strip3};

// ---- MOTOR STUFF ----

Motor m1(MOTOR_1, MOTOR_1_RMT_CHANNEL);
Motor m2(MOTOR_2, MOTOR_2_RMT_CHANNEL);
Motor m3(MOTOR_3, MOTOR_3_RMT_CHANNEL);

const long MOTOR_UPDATE_INTERVAL = 10; // motor update loop time in microseconds
long last_motor_update = micros();

// ---- LOOP STUFF ----

bool robot_enabled = false;

const long LOOP_INTERVAL = 1000; // microsec
long last_loop = micros();

// ---- CONTROLLER STUFF ----

Controller controller(CONTROLLER_SERIAL);

// movement
// range -1 to 1, negative is reverse
float x = 0;
float y = 0;

// Offset for facing direction, FIXME need values for offset
float imu_offset = 0;

// spin speed
//  range -1 to 1, negative is reverse
float spin = 0;

bool flip = false;

bool full_send = false;

// variables

float body_angle = 0;

DriveManager driveManager;

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
  // accel.init();

  // init encoders
  e1.init();
  e2.init();
  e3.init();
}

/**
   @brief Updates all sensors.
*/
void updateSensors() {
  // update IMU
  imu.update();

  // update accelerometers
  // accel.update();

  // update encoders
  e1.update();
  e2.update();
  e3.update();

  // Update values in drive manager
  float encVels[] = {e1.getUnfilteredSpeed(), e2.getUnfilteredSpeed(), e3.getUnfilteredSpeed()};
  driveManager.updateOrientation(imu.getRawVelocity(), encVels);

  //body_angle = imu.getAngle() + imu_offset;
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
void updateDrive(float spin, float x, float y) {
  // Calculate the power
  int power[3] = {0, 0, 0};
  driveManager.calcDrivePower(power, spin, x, y);

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
  digitalWrite(STATUS_LED_PIN, on);
  digitalWrite(ESP_STATUS_LED_PIN, on);
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
   @brief Updates all LED strip brightnesses.
*/
void setLEDStripBrightness(uint8_t b) {
  strip1.setBrightness(b);
  strip2.setBrightness(b);
  strip3.setBrightness(b);
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

void updateStrips(uint32_t color) {
  const float OFFSETS[] = {-PI / 3, PI / 3, PI};
  const static float angle_range = PI / 12;

  float norm_angle = driveManager.getCurAngle();

  // Iterate through strips and set values
  for (int i = 0; i < 3; i++) {
    // Flip the offset for the strip if the bot is flipped
    float offset = (flip ? -1 : 1) * OFFSETS[i];

    // Set the color
    if (abs(normalizeAngle(norm_angle + offset)) < angle_range) {
      strips[i]->fillColor(color);
    } else {
      strips[i]->fillColor(LedStrip::makeColor(0, 0, 0));
    }
  }
}

// ---- MAIN FUNCTIONS ----

void setup() {

  Serial.begin(115200);

  // Initialize the access point and OTA udpater
  Serial.print("Initializing AP...");
  bool success = initAp(AP_SSID);
  Serial.println(success ? "done!" : "fail.");
  if (success) {
    Serial.print("Initializing OTA...");
    initOta();
    Serial.println("Ready!");
  }

  initLEDs();

  // set LED strips to red
  fillLEDs(255, 0, 0);
  showLEDStrips();

  controller.init();
  controller.setJoyDeadzone(0.1);

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
        Serial.println("run");
      }
      if (controller.buttonClick(LEFT)) {
        spin = 0;
        robot_enabled = false;
        Serial.println("stop");
      }

      x = -controller.joystick(RIGHT, X); // For some stupid reason, the x-axis is inverted.
      y = controller.joystick(RIGHT, Y);

      // "Rotate". Limit rotation rate to 100deg/s.
      static unsigned long lastAngleChange = millis();
      if (millis() - lastAngleChange > 10) {
        driveManager.rotate(DEG_TO_RAD * controller.joystick(LEFT, X));
        lastAngleChange = millis();
      }

      // gyro drift comensation
      // Robot rotates at ~500rpm = 52 rad/s. 
      if (controller.dpadClick(RIGHT) && robot_enabled) {
        driveManager.adjustDrift(0.8);
      }
      if (controller.dpadClick(LEFT) && robot_enabled) {
        driveManager.adjustDrift(-0.8);
      }

      if (controller.dpadClick(UP) && robot_enabled) {
        spin += 0.1;
        if (spin > 1.0) {
          spin = 1.0;
        }
        Serial.println("spin up");
        Serial.println(spin);
      }

      if (controller.dpadClick(DOWN) && robot_enabled) {
        spin -= 0.1;
        if (spin < 0.1) {
          spin = 0.1;
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
        Serial.print("angle: \t");
        Serial.print((driveManager.getCurAngle() * RAD_TO_DEG));
        Serial.println();
      }

      if (controller.button(UP)) {
        Serial.print("accel: \t");
        Serial.print("");
        Serial.println();
      }

      if (controller.bumper(RIGHT) && robot_enabled) {
        full_send = true;
        Serial.println("FULL SEND!!!");
      } else {
        full_send = false;
      }

      // re-arm motors when power is reapplyed
      if (controller.joyButtonClick(LEFT)) {
        // set LED strips to yellow
        fillLEDs(255, 255, 0);
        showLEDStrips();

        // arm sequence
        armMotors();

        // set LED strips to green
        fillLEDs(0, 255, 0);
        showLEDStrips();
      }

    } else {

      // controller disconnected

      // set status to off and all LED strips to red
      setStatusLED(false);
      fillLEDs(255, 0, 0);

      // disable robot
      robot_enabled = false;
    }

    updateSensors();
  }

  // motor update timer
  if (micros() - last_motor_update > MOTOR_UPDATE_INTERVAL) {
    last_motor_update = micros();

    if (robot_enabled) {

      // run motors
      if (!full_send) {
        updateDrive(spin, x, y);

        // set led strips to purple
        updateStrips(LedStrip::PURPLE);

      } else {
        // full send mode
        if (flip) {
          sendAllMotorPower(-FULL_SEND_POWER);
        } else {
          sendAllMotorPower(FULL_SEND_POWER);
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

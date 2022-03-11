#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"
#include <SimpleKalmanFilter.h> // https://github.com/denyssene/SimpleKalmanFilter

class Encoder {
public:
  Encoder(int aPin, int bPin, int ticksPerRotation);

  void init();
  int estimateSpeed();
  int getSpeed();
  int getFilteredSpeed();
  void invertDirection(bool invertDir);

private:
  void tick(bool trigA);

  // Static stuff for interrupt handling
  static void isrA();
  static void isrB();
  static Encoder *instance;

  // Pins and pin registers
  int aPin, bPin;
  volatile uint32_t *aPinRegister;
  volatile uint32_t *bPinRegister;
  uint32_t aPinBit, bPinBit;

  volatile int ticks = 0; // Number of ticks since last speed estimate

  volatile int speed; // current speed of the motor in rpm
  int filteredSpeed;  // current filtered speed of the motor in rpm

  bool invertDir = false; // Track whether speed estimation should be negated

  volatile unsigned long
      lastTickTime; // time in microseconds of the last encoder tick
  volatile unsigned long lastEstTime; // time in microseconds of the tick
                                      // preceding the last estimate

  // Conversion from (ticks / us) -> (rot / min)
  int ticksPerRotation = 360;
  unsigned long tickConversion = 1000000ul * 60 / ticksPerRotation;

  /*
    SimpleKalmanFilter(e_mea, e_est, q);
    e_mea: Measurement Uncertainty
    e_est: Estimation Uncertainty
    q: Process Noise
  */
  SimpleKalmanFilter speedFilter;
};

#endif
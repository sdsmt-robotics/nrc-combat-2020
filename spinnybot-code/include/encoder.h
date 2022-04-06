#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <SimpleKalmanFilter.h>

class Encoder {
public:
  Encoder(int pin, int ticksPerRotation);

  void init();
  void update();
  int getSpeed();
  int getUnfilteredSpeed();
  float getPos();
  void resetPos();

private:
  void tick();
  // Static stuff for interrupt handling
  static unsigned instance_count;
  static void isr(unsigned instance_num);
  static Encoder **instances;

  // Pins and pin registers
  int _pin;

  volatile int tick_count = 0; // Number of ticks since last speed estimate
  volatile long totTicks = 0;  // Current position

  volatile int speed; // current speed of the motor in rpm
  int filtered_speed;

  bool invertDir = false; // Track whether speed estimation should be negated

  volatile unsigned long
      lastTickTime; // time in microseconds of the last encoder tick
  volatile unsigned long lastUpdateTime; // time in microseconds of the tick
                                         // preceding the last estimate

  // Conversion from (ticks / us) -> (rot / min)
  int _ticksPerRotation = 360;
  float _rotationsPerTick = 1 / float(_ticksPerRotation);
  unsigned long tickConversion = 1000000ul * 60 / _ticksPerRotation;

  SimpleKalmanFilter filter;
};

#endif
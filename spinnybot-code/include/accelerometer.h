#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "adc.h"
#include "filter.h"
#include <Arduino.h>

#define PI_2 2.0 * PI
#define DEG_2_RAD PI / 180
#define RAD_2_DEG 180 / PI

class ADC {
public:
  ADC(HardwareSerial &serial);
  void init(int baud, int rx, int tx);
  uint32_t readData();

private:
  ADS122U04 adc;
};

class Accelerometers {
public:
  Accelerometers(HardwareSerial &serial, int mux_pin, int rx, int tx);
  void init();
  float update();
  float getAngle();
  float getVelocity();
  void reset();

private:
  float normalizeAngle(float angle);
  void readADC(uint32_t &v1, uint32_t &v2);
  float integrate(float deltaT);

  ADC adc1;
  ADC adc2;

  Filter velocity_filter;

  int _mux_pin = 0;
  int _rx_pin = 0;
  int _tx_pin = 0;

  float angle = 0;

  // radius
  const float RADIUS = 1.0;

  long last_update = 0;

  float velocityVals[3] = {0};
  int valsMidIdx = 0;
};

#endif
#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "adc.h"
#include <Arduino.h>

class ADS122U04 {
public:
  ADS122U04(HardwareSerial *serial);
  bool init();
  float read();

private:
  SFE_ADS122C04 adc;
};

#endif
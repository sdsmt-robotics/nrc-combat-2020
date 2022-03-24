#include "accelerometer.h"

ADS122U04::ADS122U04(HardwareSerial *serial) : adc(serial) {}

bool ADS122U04::init() { return adc.begin(115200); }

float ADS122U04::read() { return adc.readADC(); }
#include "accelerometer.h"

Accelerometers::Accelerometers(HardwareSerial &serial, int mux_pin)
    : adc1(serial), adc2(serial), velocity_filter(3) {
  _mux_pin = mux_pin;
}

bool Accelerometers::init() {
  pinMode(_mux_pin, OUTPUT);
  digitalWrite(_mux_pin, LOW);
  adc1.init();
  digitalWrite(_mux_pin, HIGH);
  adc2.init();
  return false;
}

float Accelerometers::update() {
  unsigned long curTime;
  curTime = micros();

  uint32_t raw[2] = {0};
  readADC(raw[0], raw[1]);

  // calculate velocity
  float v = (sqrtf(R * raw[0]) + sqrtf(R * raw[1])) / (2.0 * R);
  velocity_filter.filter(v);

  float deltaT = float(curTime - last_update) / 1000000;
  last_update = curTime;

  integrate(deltaT);

  // Put into range [-PI, PI]
  angle = normalizeAngle(angle);
}

float Accelerometers::getVelocity() { return velocity_filter.getFilteredVal(); }

float Accelerometers::getAngle() { return angle; }

void Accelerometers::reset() { angle = 0; }

float Accelerometers::integrate(float deltaT) {
  velocityVals[valsMidIdx] = velocity_filter.getFilteredVal();
  if (valsMidIdx >= 2) {
    valsMidIdx = 0;
  }
  angle += ((velocityVals[0] + velocityVals[1] + velocityVals[2] +
             3 * velocityVals[valsMidIdx]) /
            3) *
           deltaT;
  ++valsMidIdx;
  return angle;
}

void Accelerometers::readADC(uint32_t &v1, uint32_t &v2) {
  digitalWrite(_mux_pin, LOW);
  v1 = adc1.readData();
  digitalWrite(_mux_pin, HIGH);
  v2 = adc2.readData();
}

float Accelerometers::normalizeAngle(float angle) {
  if (angle > PI) {
    do {
      angle -= PI_2;
    } while (angle > PI);
  } else {
    do {
      angle += PI_2;
    } while (angle < -PI);
  }
  return angle;
}

// ADC functions

ADC::ADC(HardwareSerial &serial) : adc(&serial) {}

bool ADC::init() {
  adc.begin(115200);
  return true;
}

uint32_t ADC::readData() { return adc.readADC(); }
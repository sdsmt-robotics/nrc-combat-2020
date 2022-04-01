#include "encoder.h"

unsigned Encoder::instance_count = 0;
Encoder **Encoder::instances = nullptr;

/**
   @brief Constructor for the class.

   @param aPin - First encoder pin. Attached to interrupt.
   @param bPin - second encoder pin.
   @param ticksPerRotation - Number of encoder ticks per rotation.
*/
Encoder::Encoder(int pin, int ticksPerRotation) : filter(100, 100, 0.08) {
  _pin = pin;
  _ticksPerRotation = ticksPerRotation;
}

/**
   @brief Initialize the encoder.
*/
void Encoder::init() {
  // Setup control pins
  pinMode(_pin, INPUT_PULLUP);

  // Set up the encoder interrupt pin
  instance_count++;
  instances =
      (Encoder **)realloc(instances, instance_count * sizeof(Encoder *));
  instances[instance_count - 1] = this;
  void (*isr)();
  switch (instance_count) {
  case 1:
    isr = []() { Encoder::isr(0); };
    break;
  case 2:
    isr = []() { Encoder::isr(1); };
    break;
  case 3:
    isr = []() { Encoder::isr(2); };
    break;
  default:
    isr = []() {};
    break;
  }
  attachInterrupt(digitalPinToInterrupt(_pin), isr, CHANGE);

  // initialize the timer
  lastTickTime = micros();
  lastUpdateTime = lastTickTime;
}

/**
   Update the current speed estimate and return the filtered value.
*/
void Encoder::update() {
  // Calculate the speed if we got a tick. Otherwise assume 0.
  if (lastTickTime != lastUpdateTime) {
    speed = tick_count * (tickConversion / (lastTickTime - lastUpdateTime));
  } else {
    speed = 0;
  }

  // Reset things
  lastUpdateTime = lastTickTime;
  tick_count = 0;

  filtered_speed = filter.updateEstimate(speed);
}

/**
   @brief Get the current speed the motor is running at in RPMs.

   @return the current speed of the motor.
*/
int Encoder::getSpeed() { return filtered_speed; }

int Encoder::getUnfilteredSpeed() { return speed; }

/**
   @brief Handle interrupt. Forward it to stored instance.
*/
void Encoder::isr(unsigned instance_num) { instances[instance_num]->tick(); }

void Encoder::tick() {
  lastTickTime = micros();
  tick_count++;
}
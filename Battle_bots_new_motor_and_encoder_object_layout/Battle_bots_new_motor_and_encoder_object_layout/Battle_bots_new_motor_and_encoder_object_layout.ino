#include "AS5134.h"
#include <BTS7960.h>

#define INIT_POWER 20
#define MAX_SPEED 30

//motor objects
BTS7960 motor_1(18, 17, 2, true);
BTS7960 motor_2(24, 23, 3, true);
BTS7960 motor_3(26, 25, 6, true);

//motor encoder objects
AS5134 motor_encoder_1(1, 15, 7);
AS5134 motor_encoder_2(1, 16, 7);
AS5134 motor_encoder_3(1, 5, 7);


void setup() {
  Serial.begin(38400);

  //Initialize the encoder
  motor_encoder_1.init();
  motor_encoder_2.init();
  motor_encoder_3.init();

  //initialize the motors
  motor_1.init();
  motor_1.stop();
  motor_1.run(INIT_POWER);
 
  motor_2.init();
  motor_2.stop();
  motor_2.run(INIT_POWER);
 
  motor_3.init();
  motor_3.stop();
  motor_3.run(INIT_POWER);
}


void loop() {
  // put your main code here, to run repeatedly:

}

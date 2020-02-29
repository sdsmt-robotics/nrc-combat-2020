#include <BTS7960.h>
#include <PS3BT.h>
#include <Wire.h>
//#include <LIS3MDL.h>
#include <math.h>

float avg = 0;
float setpoint, theta;

//Motor 1
const int REN1 = 45;
const int LEN1 = 47;
const int PWM1 = 11;

//Motor 2
const int REN2 = 41;
const int LEN2 = 43;
const int PWM2 = 5;

//Motor 3
const int REN3 = 37;
const int LEN3 = 39;
const int PWM3 = 6;

//LIS3MDL mag;
USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

int x, y, xp, yp, r;
int w1s, w2s, w3s;
int w1sOld, w2sOld, w3sOld;
unsigned long lastUpdate = 0;

void setup() {
  if (Usb.Init() == -1) {
    while (1); //halt
  }
  // put your setup code here, to run once:

  pinMode(REN3,OUTPUT);
  pinMode(LEN3,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(REN2,OUTPUT);
  pinMode(LEN2,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(REN1,OUTPUT);
  pinMode(LEN1,OUTPUT);
  pinMode(PWM1,OUTPUT);
  
  Serial.begin(115200);

  Wire.begin();

  //if (!mag.init())
  //{
  //    Serial.println("Failed to detect and initialize magnetometer!");
  //    while (1);
  //}
  //mag.enableDefault();
  
  //for(int i = 0; i < 5; i++)
  //{
  //    mag.read();
  //    float angle = atan2(x, y) + PI;
  //    avg += angle;
  //    delay(100);
  //}
  //avg = avg / 5;
  //setpoint = avg;
}

void loop() {
  Usb.Task();
  if (PS3.PS3Connected)
  {
    //grab the analog value from the joystick and run it through the
    //  joyToPWM function to translate it to something usefull.
    //x = joyToPWM(PS3.getAnalogHat(RightHatX));
    //y = joyToPWM(PS3.getAnalogHat(RightHatY));
    xp = joyToPWM(PS3.getAnalogHat(LeftHatX));
    yp = -1 * joyToPWM(PS3.getAnalogHat(LeftHatY));
    r = joyToPWM(PS3.getAnalogHat(RightHatX));

    //mag.read();
    //float mag_x = mag.m.x - 5850;
    //float mag_y = mag.m.y + 6750;
    //float angle = atan2(mag_x, mag_y) + PI;      
    //theta = setpoint - angle;
    //if(theta < 0)
    //theta = theta + 2 * PI;
       
    //x = xp*sin(theta) + yp*cos(theta);
    //y = xp*cos(theta) - yp*sin(theta);
    
    //set the speed and direction of the motors using the L289N library
    w1s = -0.5 * xp - sqrt(3)/2 * yp + r;
    w2s = -0.5 * xp + sqrt(3)/2 * yp + r;
    w3s = xp + r;

    //limit speed
    w1s *= 8;    
    w2s *= 8; 
    w3s *= 8;


    if(w1sOld != w1s)
    {
      w1sOld = w1s;
      lastUpdate = millis();
      Wire.beginTransmission(1); // transmit to device #1      // sends five bytes
      Wire.write(w1s);              // sends one byte  
      Wire.write((w1s)>>8);  
      Wire.endTransmission();    // stop transmitting
    }
    
    if(w2sOld != w2s)
    {
      w2sOld = w2s;
      lastUpdate = millis();
      Wire.beginTransmission(2); // transmit to device #2      // sends five bytes
      Wire.write(w2s);              // sends one byte  
      Wire.write((w2s)>>8);  
      Wire.endTransmission();    // stop transmitting
    }
  
    if(w3Old != w3s)
    {
      w3sOld = w3s;
      lastUpdate = millis();
      Wire.beginTransmission(3); // transmit to device #3      // sends five bytes
      Wire.write(w3s);              // sends one byte  
      Wire.write((w3s)>>8);  
      Wire.endTransmission();    // stop transmitting
    }

    if((millis() - lastUpdate) > 100))
    {
      Wire.beginTransmission(3); // transmit to device #3      // sends five bytes
      Wire.write(w3s);              // sends one byte  
      Wire.write((w3s)>>8);  
      Wire.endTransmission();    // stop transmitting
    }

    delay(10);
  }
  else // set speed to zero if controller is not connected
  {
    Wire.beginTransmission(1); // transmit to device #1      // sends five bytes
    Wire.write(0);              // sends one byte  
    Wire.write((0)>>8);  
    Wire.endTransmission();    // stop transmitting
    Wire.beginTransmission(2); // transmit to device #2      // sends five bytes
    Wire.write(0);              // sends one byte  
    Wire.write((0)>>8);  
    Wire.endTransmission();    // stop transmitting
    Wire.beginTransmission(3); // transmit to device #3      // sends five bytes
    Wire.write(0);              // sends one byte  
    Wire.write((0)>>8);  
    Wire.endTransmission();    // stop transmitting
  }
}

//a function to map a raw joystick value to a value that works with
//  the L289N library
int joyToPWM(int joyVal)
{
  int preCurve;
  int postCurve;

  //if joyVal is >137 or <117, it's outside of the deadzone and
  // power should be send to the motors
  if (joyVal > 137)
  {
    preCurve = -(map(joyVal, 137, 255, 0, 255));
  }
  else if (joyVal <  117)
  {
    //the arduino map function can't invert a value so I wrote my own math
    preCurve = (-2.18*joyVal)+255;
  }
  //otherwise, it must be in the deadzone and the motors should not
  // receive power
  else
  {
    return 0;
  }

  //apply an easing curve to the value
  postCurve = pow(2.718, 0.0198 * abs(preCurve)) + 99;

  //the curve doesn't handle negative values, manually negate if needed
  if (preCurve < 0)
    postCurve = -postCurve;

  return postCurve;
}

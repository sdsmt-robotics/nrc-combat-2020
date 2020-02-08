#include <BTS7960.h>
#include <PS3BT.h>
#include <Wire.h>
//#include <LIS3MDL.h>
#include <math.h>

float avg = 0;
float setpoint, theta;

//LIS3MDL mag;
USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

BTS7960 w1(9, 39, 37, true);
BTS7960 w2(10, 43, 41, true);
BTS7960 w3(11, 47, 45, true);

int x, y, xp, yp, r;
int w1s, w2s, w3s;

void setup() {
  if (Usb.Init() == -1) {
    while (1); //halt
  }

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
  Usb.Task(); //this wil manage the controller connection

  if (PS3.PS3Connected) //only run the drive code if the controller is connected
  {
      //grab the analog value from the joystick and run it through the
      //  joyToPWM function to translate it to something the L289N library
      //  can use
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
      w3s = x + r;

      w1s *= 0.35;
      w2s *= 0.35;
      w3s *= 0.35;

      Serial.println(w1s);
      Serial.println(w2s);
      Serial.println(w3s);

    if(w1s < 0)
    {
      
    }
    
    if(w2s < 0)
    {
  
    }
  
    if(w3s < 0)
    {
      
    }
  
  }
  else
  {
    Serial.println("No controller connected. \n");
    delay(500);
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

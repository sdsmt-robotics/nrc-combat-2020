#include <BTS7960.h>
#include <PS3BT.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <math.h>

const float m1Offset = ( (2*PI)/3);
const float m2Offset = ( (4*PI)/3);

LIS3MDL mag;
USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

unsigned long lastUpdate = 0;
unsigned long timeDiff;

float maxx = 0, maxy =0 , minx =0, miny=0 ;

float setpoint, theta;

void setup() {

  Serial.begin(115200);

  Wire.begin();
  
  if (Usb.Init() == -1) 
  {
      while (true)
      {
        //Serial.println("Failed to detect and initialize USB!");
        delay(500);
      }
  }

  if (!mag.init())
  {
      while (true)
      {
        //Serial.println("Failed to detect and initialize magnetometer!");
        delay(500);
      }
  }
  
  mag.enableDefault();
  
  float avg = 0;
  
  for(int i = 0; i < 60; i++)
  {
    mag.read();
    float mag_x = mag.m.x + 6501;// - 5850;
    float mag_y = mag.m.y; + 4753;// + 6750;
    float angle = atan2(mag_x, mag_y); 
    avg += angle;
    delay(50);
    //Serial.println("test");
  }
  avg = avg / 60;
  setpoint = avg;
}

void loop() {
  int xp, yp, r;
  int w1s, w2s, w3s;
  Usb.Task();
  
  if (PS3.PS3Connected)
  {
    
    //grab the analog value from the joystick and run it through the
    //joyToPWM function to translate it to something usefull.
    xp = joyToPWM(PS3.getAnalogHat(LeftHatX));
    yp = -1 * joyToPWM(PS3.getAnalogHat(LeftHatY));
    r = joyToPWM(PS3.getAnalogHat(RightHatX));

    mag.read();
    float mag_x = mag.m.x;
    float mag_y = mag.m.y;


    maxx = max(maxx,mag_x);
    maxy = max(maxy,mag_y);
    minx = min(minx,mag_x);
    miny = min(miny,mag_y);

    mag_x = (mag_x-(minx + maxx)/2.0)/(maxx-minx);
    mag_y = (mag_y-(miny + maxy)/2.0)/(maxy-miny);

    //debug
    //Serial.println("X");
    //Serial.println(maxx);
    //Serial.println(minx);
    //Serial.println("Y");
    //Serial.println(maxy);
    //Serial.println(miny);

    //Serial.print(mag_x);
    //Serial.print(",");
    //Serial.println(mag_y);
    
    //compute angle diff betwean current
    theta = atan2(mag_x, mag_y) - setpoint;
    if(theta < 0)
      theta = theta + 2 * PI;
    
    //Serial.println(theta*200);

    //set the velocity of the motors
    w1s = r + sin(theta - m2Offset)*yp - cos(theta-m2Offset)*xp;
    w2s = r + sin(theta - m1Offset)*yp - cos(theta-m1Offset)*xp;
    w3s = r + sin(theta)*yp - cos(theta)*xp;

    //limit speed
    w1s *= 1.5;    
    w2s *= 1.5; 
    w3s *= 1.5;

    //motor 1
    Wire.beginTransmission(1); // transmit to device #3      // sends five bytes
    Wire.write(w1s);              // sends one byte  
    Wire.write((w1s)>>8);  
    Wire.endTransmission();    // stop transmitting

    //motor 2
    Wire.beginTransmission(2); // transmit to device #3      // sends five bytes
    Wire.write(w2s);              // sends one byte  
    Wire.write((w2s)>>8);  
    Wire.endTransmission();    // stop transmitting

    //motor 3
    Wire.beginTransmission(3); // transmit to device #3      // sends five bytes
    Wire.write(w3s);              // sends one byte  
    Wire.write((w3s)>>8);  
    Wire.endTransmission();    // stop transmitting

    //debug
    Serial.print(theta*63.66);
    Serial.print(',');
    Serial.print(w1s);
    Serial.print(',');
    Serial.print(w2s);
    Serial.print(',');
    Serial.println(w3s);
//    
//    theta += (timeDiff*0.0418879023)/100;
//    if(theta > (2*PI))
//      theta -= 2*PI;
//    delay(10);
      
  }
  else // set speed to zero if controller is not connected
  {
    //Wire.beginTransmission(1); // transmit to device #1      // sends five bytes
    //Wire.write(0);              // sends one byte  
    //Wire.write((0)>>8);  
    //Wire.endTransmission();    // stop transmitting
    //Wire.beginTransmission(2); // transmit to device #2      // sends five bytes
    //Wire.write(0);              // sends one byte  
    //Wire.write((0)>>8);  
    //Wire.endTransmission();    // stop transmitting
    //Wire.beginTransmission(3); // transmit to device #3      // sends five bytes
    //Wire.write(0);              // sends one byte  
    //Wire.write((0)>>8);  
    //Wire.endTransmission();    // stop transmitting
    //Serial.println("Failed to detect PS3.");
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

#include <BTS7960.h>
#include <PS3BT.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <math.h>

float avg = 0;
float setpoint, theta;

LIS3MDL mag;
USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

int xp, yp, r;
int w1s, w2s, w3s;
int w1sOld, w2sOld, w3sOld;
unsigned long lastUpdate = 0;
float maxx = 0, maxy =0 , minx =0, miny=0 ;
void setup() {
  if (Usb.Init() == -1) {
    while (1); //halt
  }
  // put your setup code here, to run once:
  
  Serial.begin(115200);

  Wire.begin();

  if (!mag.init())
  {
      Serial.println("Failed to detect and initialize magnetometer!");
      while (1);
  }
  mag.enableDefault();
  
  for(int i = 0; i < 5; i++)
  {
    mag.read();
    float mag_x = mag.m.x + 6501;// - 5850;
    float mag_y = mag.m.y; + 4753;// + 6750;
    float angle = atan2(mag_x, mag_y) + PI; 
    avg += angle;
    delay(100);
  }
  avg = avg / 5;
  setpoint = avg;
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

    mag.read();
    float mag_x = mag.m.x + 6501;
    float mag_y = mag.m.y + 4753;
    //maxx = max(maxx,mag_x);
    //maxy = max(maxy,mag_y);
    //minx = min(minx,mag_x);
    //miny = min(miny,mag_y);
    //Serial.println(maxx);
    //Serial.println(minx);
    //Serial.println(maxy);
    //Serial.println(miny);
    Serial.print(mag.m.x);
    Serial.print(',');
    Serial.println(mag.m.y);

    float angle = atan2(mag_x, mag_y) + PI;      
    theta = setpoint - angle;
    if(theta < 0)
    theta = theta + 2 * PI;
       
    xp = xp*sin(theta) + yp*cos(theta);
    yp = xp*cos(theta) - yp*sin(theta);
    
    //Serial.println(theta);

    //set the speed and direction of the motors using the L289N library
    w1s = -0.5 * xp - sqrt(3)/2 * yp + r;
    w2s = -0.5 * xp + sqrt(3)/2 * yp + r;
    w3s = xp + r;

    //limit speed
    w1s *= 1.5;    
    w2s *= 1.5; 
    w3s *= 1.5;
    
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
  
    if(w3sOld != w3s)
    {
      w3sOld = w3s;
      lastUpdate = millis();
      Wire.beginTransmission(3); // transmit to device #3      // sends five bytes
      Wire.write(w3s);              // sends one byte  
      Wire.write((w3s)>>8);  
      Wire.endTransmission();    // stop transmitting
    }

    if((millis() - lastUpdate) > 100)
    {
      Wire.beginTransmission(1); // transmit to device #3      // sends five bytes
      Wire.write(w1s);              // sends one byte  
      Wire.write((w1s)>>8);  
      Wire.endTransmission();    // stop transmitting
      Wire.beginTransmission(2); // transmit to device #3      // sends five bytes
      Wire.write(w2s);              // sends one byte  
      Wire.write((w2s)>>8);  
      Wire.endTransmission();    // stop transmitting
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

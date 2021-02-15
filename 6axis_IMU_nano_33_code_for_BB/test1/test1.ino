#include <FastLED.h>
#include <Arduino_LSM6DS3.h>

// How many leds in your strip?
#define NUM_LEDS 8

// Pin for data to led strip
#define DATA_PIN 5

const bool debug = false;
const int debug_level = 1; //0 to 4
const bool use_led = true;

//the output from 1 gyro rotaion to 360 degrees
const int gyro_to_degree = 420;

//the number of adressible horizontal stripes of leds around the robot
const int bot_resolution = 50;

//the number of adressible radial led's per horizontal stripe
CRGB leds[NUM_LEDS];

//average LMU vals
float IMU_avg_vals[6] = {0,0,0,0,0,0}; //aX, aY, aZ, gX, g

struct stripe
{
   stripe()
   {
      int i = 0;
      
      for(i;i<NUM_LEDS;++i)
      {
        pixels[i] = CRGB::Black;
      }
      
      return;
   }

   CRGB pixels[NUM_LEDS];
};

//this class manages the virtual screens used for animations and backgrounds of the display.
class screen
{
  public:
 
    //input the resolution in number coulombs of the screen as well as the total coulombs stored
    screen(int resolution_in, int stored_in)
    {

      //sore inputs
      stored = stored_in;
      resolution = resolution_in;
      
      //make a new array of stripe structures of the total size of stored
      stripe* temp = new stripe[stored];

      //see if the new array is valid and stop the program if it is not
      while(temp == NULL)
      {
        Serial.println("Failed get memory for stripe array!");
        delay(2000);
      }

      //update the coulombs pointer
      coulombs = temp;

      return;
    }

    //get the pointer to the relevant stripe
    CRGB* get_columb(int c)
    {
      //if the input is invalid, return a blank stripe
      if(c >= resolution)
      {
        return blank_stripe.pixels;
      }

      //return the correct stripe
      return (coulombs[c]).pixels;
    }
    
    //set the indicated columb
    void set_columb(int c, CRGB columb_pixel[NUM_LEDS])
    {
      int i = 0;

      //if c is valid, update the corresponding strip
      if(c < stored)
      {
        for(i; i < NUM_LEDS; ++i)
        {  
          (coulombs[c]).pixels[i] = columb_pixel[i];
        }
      }

      return;      
    }
 
   private:
   stripe *coulombs; //pointer to the array of strips
   int resolution; //the number of strips that should be displayed at once
   int stored; //the total number of strips
   int scan_start = 0; //where the scan in stored starts for the desired resolution
   stripe blank_stripe; //the blank stripe for this screen (background)
};

//construct screens(class is a work in progress)
screen  main_screen(bot_resolution,bot_resolution);

//**********setup**********
void setup() {
  
  //generic counter var
  int i = 0;
  int j = 0;

  
  //**********General setup**********
  //add leds for the fast led library
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed

  // start serial if debug is true
  if(debug)
  {
    //wait for serial to connect
    Serial.begin(9600);
    while (!Serial && debug) {}
  }

  //test for imu initialization
  if (!IMU.begin())
  {
    if(debug)
    {
      Serial.println("Failed to initialize IMU!");
    }

    // halt program
    while(true) {}
  }

  //**********Testing classes and LED's**********
  stripe TEST();
  
  screen TEST2(bot_resolution,bot_resolution);

  CRGB temp2[NUM_LEDS] = {CRGB::Blue,CRGB::Blue,CRGB::Blue,CRGB::Blue,CRGB::Blue,CRGB::Blue,CRGB::Blue,CRGB::Blue};
  
  CRGB* temp = TEST2.get_columb(1);

  for(i = 0; i < NUM_LEDS; ++i)
  {
    leds[i] = temp[i];
  }

  FastLED.show();

  
  delay(500);

  TEST2.set_columb(2,temp2);

  temp = TEST2.get_columb(2);

  for(i = 0; i < NUM_LEDS; ++i)
  {
    leds[i] = temp[i];
  }

  FastLED.show();

  delay(500);
  
  if(debug && Serial.println("Test Passed")){}

  FastLED.clear();  // clear all pixel data
  FastLED.show();
  
  //**********IMU setup**********
  //find imu static average values

  //IMU variables
  float IMUvals[6] = {0,0,0,0,0,0}; //aX, aY, aZ, gX, gY, gZ
 
  while(j < 100)
  {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable())
    {      
      IMU.readAcceleration(IMUvals[0], IMUvals[1], IMUvals[2]);
      IMU.readGyroscope(IMUvals[3], IMUvals[4], IMUvals[5]);
      for(i=0; i < 6; ++i)
      {
        IMU_avg_vals[i] = IMU_avg_vals[i] + IMUvals[i];
      }
      ++j;
    }
  }
  for(i = 0; i < 6; ++i)
  {
    IMU_avg_vals[i] = IMU_avg_vals[i]/100;
    if(debug && Serial.println(IMU_avg_vals[i])) {}
  }

  //**********LED screen setup**********
  CRGB temp_blue[NUM_LEDS] = {CRGB::Blue,CRGB::Blue,CRGB::Blue,CRGB::Blue,CRGB::Blue,CRGB::Blue,CRGB::Blue,CRGB::Blue};
  CRGB temp_red[NUM_LEDS] = {CRGB::Red,CRGB::Red,CRGB::Red,CRGB::Red,CRGB::Red,CRGB::Red,CRGB::Red,CRGB::Red};
  CRGB temp_green[NUM_LEDS] = {CRGB::Green,CRGB::Green,CRGB::Green,CRGB::Green,CRGB::Green,CRGB::Green,CRGB::Green,CRGB::Green};
  CRGB temp_yellow[NUM_LEDS] = {CRGB::Yellow,CRGB::Yellow,CRGB::Yellow,CRGB::Yellow,CRGB::Yellow,CRGB::Yellow,CRGB::Yellow,CRGB::Yellow};
  
  main_screen.set_columb(0,temp_blue);
  main_screen.set_columb((bot_resolution/4)-1,temp_red);
  main_screen.set_columb((bot_resolution/2)-1,temp_green);
  main_screen.set_columb((bot_resolution/4*3)-1,temp_yellow);

  if(debug && Serial.println("Setup compleat")){}
}

//**********main loop**********
void loop() {

  //**********variable setup**********
  //record the start time at the beginning of each loop
  unsigned long Loop_start_time = micros();
 
  //generic counter var
  int i = 0;

  //imu variables
  float IMUvals[6] = {0,0,0,0,0,0}; //aX, aY, aZ, gX, gY, gZ
  float IMU_old_vals[6] = {0,0,0,0,0,0}; //aX, aY, aZ, gX, gY, gZ

  //simpsons 1/3 variables
  static int mid = 1;
  static float vals[3] = {0,0,0};
  static float integral = 0;
  static float velocity = 0;
 
  //time between measurements (should be based off of an internal timer)
  static float delta_t = .01;

  float screen_step = 360/float(bot_resolution);
  static int screen_point = 0;

  static float degree = 0;


  //**********imu get**********

  //get raw IMU values
  if ( IMU.gyroscopeAvailable()) // IMU.accelerationAvailable() &&
  {      
    //IMU.readAcceleration(IMUvals[0], IMUvals[1], IMUvals[2]);
    IMU.readGyroscope(IMUvals[3], IMUvals[4], IMUvals[5]);
    for(i=0;(i<6) && debug && debug_level < 1; ++i)
    {
      Serial.print(IMUvals[i]);
      Serial.print(" ");
    }
    
    if(debug && debug_level < 2 && Serial.println("*")) {}
  

    //**********signal conditioning**********
  
    //abstract static average values
    for(i = 0; i < 6; ++i)
    {
      IMUvals[i] = IMUvals[i] - IMU_avg_vals[i];
    }
    //fix gravity
    IMUvals[2] = IMUvals[2] + 1;
   
    //**********numerical integration**********
    //based off of simpsons 1/3 to find the approximate angle
  
    if(mid > 2)
    {
      mid = 0;
      vals[2] = IMUvals[5];
    }
    else if(mid == 0)
    {
      vals[0] = IMUvals[5];
    }
    else
    {
      vals[1] = IMUvals[5];
    }
     
    velocity = ((vals[0] + vals[1] + vals[2] + vals[mid])/2);
  
    ++mid;
  
  }

  integral = integral + velocity*delta_t;
  
  //**********integral to rad**********
  if (integral > gyro_to_degree)
  {
    integral -= gyro_to_degree;
  }
  else if ((gyro_to_degree+integral) < 0)
  {
    integral += gyro_to_degree;
  }
 
  //convert the integral to degree
  degree = float(360)/gyro_to_degree*integral;

  //debug outputs
  if(debug && debug_level < 3 && Serial.print(integral)) {Serial.print(" ");}
  if(debug && debug_level < 4 && Serial.println(degree)) {}

  // led code
  if(use_led)
  {
    if (degree-screen_step > screen_step*screen_point)
    {
      
      CRGB* temp = main_screen.get_columb(screen_point);
      
      for(i = 0; i< NUM_LEDS; ++i)
      {
        leds[i] = temp[i];
      }
      
      screen_point++;
      FastLED.show();
      
    }
    else if(screen_step*screen_point > (degree+screen_step))
    {
      screen_point = 0;
    }

      if(debug && debug_level < 3 && Serial.print("Screen point:")) 
      {Serial.println(screen_point);}
  }  
  
  //**********end loop**********
  //update the time of the loop based on the last loop
  delta_t = float(micros() - Loop_start_time)/1000000;
 
}

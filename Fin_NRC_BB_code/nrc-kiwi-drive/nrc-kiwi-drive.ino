//#include <PS3BT.h> (controller library)
#include <math.h>
#include <FastLED.h>
#include <Arduino_LSM6DS3.h>
#include <SPI.h>
#include "/home/joseph/Desktop/Robot/NRC/MotorSpeedController/Software/controllerInterfaceLib/Nidec24hController.cpp" //?
#include "/home/joseph/Desktop/Robot/NRC/Controller/Code/receive/Controller.cpp" //?

//**********constants for the motors**********

//radian offsets used for the first and second motors 
const float m1Offset = ( (2*PI)/3);
const float m2Offset = ( (4*PI)/3);

//the output from 1 gyro rotaion
const int gyro_to_rad = 420;

//constant rotaiononal speed for the bot
const float rotation_speed = 400;

//**********motor objects**********

// Pin for motor slaves
#define MOTOR_PIN_1 5
#define MOTOR_PIN_2 6
#define MOTOR_PIN_3 7

// Motor Controller class (check that i am using this right
Nidec24hController motor1(SPI, MOTOR_PIN_1);
Nidec24hController motor2(SPI, MOTOR_PIN_2);
Nidec24hController motor3(SPI, MOTOR_PIN_3);

//**********Controller object and key binding**********

//Create the communications object. Use Serial for the communications.
Controller controller(Serial1);

#define START_BUTTON UP
#define STOP_BUTTON DOWN
#define DRIVE_JOYSTICK LEFT

//**********constants for the LEDs********** (move some of this to a seperate file and class)

//the number of adressible horizontal stripes of leds around the robot
const int bot_resolution = 50;

// How many leds in your strip?
#define NUM_LEDS 8

// Pin for data to led strip
#define DATA_PIN 4

//the number of adressible radial led's per horizontal stripe
CRGB leds[NUM_LEDS];

const float screen_step = (2*PI)/float(bot_resolution);

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


//**********Other global vars and consts**********

//average LMU vals
float IMU_avg_vals[6] = {0,0,0,0,0,0}; //aX, aY, aZ, gX, g

//set up ps3 controller related objects (to be removed/repalced)
//USB Usb;
//BTD Btd(&Usb);
//PS3BT PS3(&Btd);

const bool debug = false;
const int debug_level = 1; //0 to 4
const bool use_led = true;

//**********setup**********
void setup() {
  //generic counter var
  int i = 0;
  int j = 0;
  
  //**********SPI setup**********
  // Initialize the SPI communications
  SPI.begin ();
  SPI.setClockDivider(SPI_CLOCK_DIV4);

  //**********motor setup**********
  // Initialize the motor control
  motor1.init();
  motor2.init();
  motor3.init();

  motor1.setPower(100);
  motor2.setPower(100);
  motor3.setPower(100);

  motor1.brake();
  motor2.brake();
  motor3.brake();

  //**********controller setup**********
  
  //initlise the controller class
  controller.init();

  if(debug && Serial.println("Waiting for connection..."));

  //wait for inital controller conection
  while (!controller.connected()) { delay(10); }
  
  if(debug && Serial.println("Connected..."));

  //set a deadzone for the joysticks
  controller.setJoyDeadzone(0.08);
  
  //**********controller setup**********
  if (false)//Usb.Init() == -1) 
  {
      while (true)
      {
        Serial.println("Failed to detect and initialize USB!");
        delay(500);
      }
  }
  
  //**********imu initialization**********
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
  
  //add leds for the fast led library
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
    
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
  float xp, yp; //the instructed values for x & y movment
  float w1s, w2s, w3s;// the rotaional velocity set for each motor
  const int x_y_limit = 100; //limit the x/y velocity by a specifyed %

  float theta = 0; //the angle of the bot

  int i = 0;//generic counter var

  //imu variables
  float IMUvals[6] = {0,0,0,0,0,0}; //aX, aY, aZ, gX, gY, gZ
  float IMU_old_vals[6] = {0,0,0,0,0,0}; //aX, aY, aZ, gX, gY, gZ

  //simpsons 1/3 variables
  int mid = 1;
  float vals[3] = {0,0,0};
  float integral = 0;
  float velocity = 0;

  //time between measurements (should be based off of an internal timer)
  float delta_t = .01;

  int screen_point = 0; //row of the screan being displayed ///////////

  //time recording vars
  unsigned long Loop_start_time = 0;
  unsigned  long time_at_controller_loss = 0;
  
  //Should the main code run?
  bool run_mode = false;

  if(controller.connected() && controller.buttonClick(START_BUTTON))
  {
    run_mode = true;
  }
  
  while(run_mode)
  {

    //record the start time at the beginning of each loop
    Loop_start_time = micros();

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

      //scan through the array to add the new value
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

      //compute simpsons 1/3 for the past three values without multiplying by delta t
      velocity = ((vals[0] + vals[1] + vals[2] + vals[mid])/2);

      //update the new mid of the array
      ++mid;
      
    }

    //update the intigral (rotaion postion) based on the last known average velocity and delta_t 
    integral = integral + velocity*delta_t;
      
    //**********integral to rad**********
    if (integral > gyro_to_rad)
    {
      integral -= gyro_to_rad;
    }
    else if ((gyro_to_rad+integral) < 0)
    {
      integral += gyro_to_rad;
    }
     
    //convert the integral to radians
    theta = (PI*2)/gyro_to_rad*integral;

    //debug outputs
    if(debug && debug_level < 3 && Serial.print(integral)) {Serial.print(" ");}
    if(debug && debug_level < 4 && Serial.println(theta)) {}
      
    //check if controoler is conected
    if (controller.connected())
    {

      //**********controller get**********
      
      //grab the analog value from the joystick and scalil it to 100%

      xp = controller.joystick(DRIVE_JOYSTICK,X);
      yp = controller.joystick(DRIVE_JOYSTICK,Y);

      if(controller.button(STOP_BUTTON))
      {
        run_mode = false;
      }

      //**********update motor speeds**********
      
      //set the velocity of the motors
      w1s = rotation_speed + x_y_limit*(sin(theta - m2Offset)*yp - cos(theta-m2Offset)*xp)/100;
      w2s = rotation_speed + x_y_limit*(sin(theta - m1Offset)*yp - cos(theta-m1Offset)*xp)/100;
      w3s = rotation_speed + x_y_limit*(sin(theta)*yp - cos(theta)*xp)/100;

      //update time for tracking when the controller is lost
      time_at_controller_loss = Loop_start_time;
    }
    else // set speed to zero if controller is not connected
    {
      //if(debug)
      //  Serial.println("Failed to detect PS3.");

      //**********update motor speeds for no controller**********
      
      //set the velocity of the motors
      w1s = rotation_speed;
      w2s = rotation_speed;
      w3s = rotation_speed;
      
      //run_mode (update based on time delay)
      if(time_at_controller_loss > (Loop_start_time - 1000000)) //one second
      {
         run_mode = false;
      }
    }

    //**********send out motor speeds**********
    motor1.setSpeed(w1s);
    motor2.setSpeed(w2s);
    motor3.setSpeed(w3s);
    
    //debug
    if(debug && debug_level < 2)
    {
      Serial.print(w1s);
      Serial.print(',');
      Serial.print(w2s);
      Serial.print(',');
      Serial.println(w3s);
    }

    //**********LED's**********
    if(use_led)
    {
      if (theta-screen_step > screen_step*screen_point)
      {
        
        CRGB* temp = main_screen.get_columb(screen_point);
        
        for(i = 0; i< NUM_LEDS; ++i)
        {
          leds[i] = temp[i];
        }
        
        screen_point++;
        FastLED.show();
        
      }
      else if(screen_step*screen_point > (theta+screen_step))
      {
        screen_point = 0;
      }

      if(debug && debug_level < 3 && Serial.print("Screen point:")) 
      {Serial.println(screen_point);}
    }  
  }

  //set motor speed to zero as the bot is nolonger in run mode
  
    motor1.brake();
    motor2.brake();
    motor3.brake();
}

/************************************************************************/ /**
*@file
*@brief The main code to load onto the NRC 2021 battle bot
*
*TO DO
* - move led stuff into its own class
* - figure out what is wrong with the last two includes
* - test on hardware
* - clean up code more
* - figure out how to store and load the images for the leds
* - debug
***************************************************************************/

#include <math.h>
#include <FastLED.h>
#include <SPI.h>
#include "/home/joseph/Desktop/Robot/NRC/MotorSpeedController/Software/controllerInterfaceLib/Nidec24hController.cpp" //?
#include "/home/joseph/Desktop/Robot/NRC/Controller/Code/receive/Controller.cpp"                                      //?
#include "/home/joseph/Desktop/Robot/NRC/nrc-combat-2020/Fin_NRC_BB_code/POV_Display.cpp"                             //?
#include "/home/joseph/Desktop/Robot/NRC/nrc-combat-2020/Fin_NRC_BB_code/BB_IMU.cpp"                                  //?

/*********************Pin labels***********************
 * 0 for serial to the xBee REQ
 * 1 for serial to the xBee REQ
 * 2 SS for the slave motor 2
 * 3 SS for the slave motor 3
 * 4 for the led strips 2
 * 5 for the led strips 3
 * 6
 * 7 SS for the slave motor
 * 8
 * 9
 * 10 SS for the slave motor 1
 * 11 for SPI to the motor boards REQ
 * 12 for SPI to the motor boards REQ
 * 13 for SPI to the motor boards REQ
 * A0/14
 * A1/15
 * A2/16
 * A3/17 for the led strips 1
 * A4/18
 * A5/19 
 * A6/20
 * A7/21
 * 
 * GPIO 1
 * GPIO 2
 * GPIO 3
 * GPIO 4
 * GPIO 5
 * GPIO 6
 * GPIO 7
 * GPIO 8
 * GPIO 9
 * GPIO 10
 * GPIO 11
 * GPIO 12
 * GPIO 13
 * GPIO 14
 * GPIO 15
 * GPIO 16
 * GPIO 17
 * GPIO 18
 * GPIO 19
 * GPIO 20
 * GPIO 21
 * GPIO 22
 * GPIO 23
 * GPIO 24
 * GPIO 25
 * GPIO 26
 * GPIO 27
 * GPIO 28
 * GPIO 29
 * GPIO 30
 * GPIO 31
 * GPIO 32
 * GPIO 33
 * GPIO 34
 * GPIO 35
 * GPIO 36
 * GPIO 37
 * GPIO 38
 * GPIO 39
 ***************************************************/

//**********constants for the motors**********
const float rotation = 300; // Chassis rotation speed in RPM
const float translation = 1.5; // Driving feet per sec
const float chassisRad = 6; // Chassis radius in inches
const float wheelRad = 1; // Wheel radius in inches
const float translateSpeed = translation / wheelRad * (12 * 60 / 2 / PI);
const float rotationSpeed = rotation * chassisRad / wheelRad;

//**********motor objects**********

// Pin for motor slaves
#define MOTOR_PIN_1 SS
#define MOTOR_PIN_2 2
#define MOTOR_PIN_3 3

// Motor Controller class
Nidec24hController motor1(SPI, MOTOR_PIN_1);
Nidec24hController motor2(SPI, MOTOR_PIN_2);
Nidec24hController motor3(SPI, MOTOR_PIN_3);

//**********Controller object and key binding**********

//Create the communications object. Use Serial for the communications.
Controller controller(Serial1);

#define START_BUTTON UP //button
#define STOP_BUTTON DOWN //button
#define DRIVE_JOYSTICK LEFT //joystick
#define TURN_JOYSTICK RIGHT //joystick
#define PHASE_LEAD RIGHT //button
#define PHASE_LAG LEFT //button
#define SPEED_UP RIGHT //trigger
#define SPEED_DOWN LEFT //trigger

//**********constants for the LEDs**********

//the number of adressible horizontal stripes of leds around the robot
const int bot_resolution = 50;

// How many leds in your strip?
#define NUM_LEDS 8

// Pin for data to led strip
#define DATA_PIN_1 17
#define DATA_PIN_2 4
#define DATA_PIN_3 5

//the number of addressable radial led's per horizontal stripe
CRGB leds[NUM_LEDS*3];

const float screen_step = (2 * PI) / float(bot_resolution);

//construct screens(class is a work in progress)
//screen main_screen(bot_resolution, NUM_LEDS, bot_resolution);

//**********DATA for the LEDs********** (continued in setup)
//these arrays contain all data for static led patterns and simple 
//animations not defined by a helper function

CRGB red[NUM_LEDS];
CRGB orange[NUM_LEDS];
CRGB yellow[NUM_LEDS];
CRGB green[NUM_LEDS];
CRGB blue[NUM_LEDS];
CRGB purple[NUM_LEDS];

//**********Other global vars and consts**********

const bool debug = false;
const int debug_level = 1; //0 to 4
const bool use_led = true;

bb_imu orientation;

//**********setup**********
/** ***************************************************************************
* @par Setup:
* Sets up any objects that need initialization, makes tests for valid
* configurations, and finds global variables that then remain constant throughout
* the rest of the code.
*
 *****************************************************************************/
void setup()
{
    //generic counter var
    int i = 0;
    int j = 0;

    // start serial if debug is true
    if (debug)
    {
        //wait for serial to connect
        Serial.begin(9600);
        while (!Serial && debug)
        {
        }
    }

    if (debug)
    {
        Serial.println("Serial start");
    }

    //**********Testing classes and LED's**********

    //add leds for the fast led library
    FastLED.addLeds<NEOPIXEL, DATA_PIN_1>(leds, 0, NUM_LEDS); // GRB ordering is assumed
    FastLED.addLeds<NEOPIXEL, DATA_PIN_2>(leds, NUM_LEDS, NUM_LEDS); // GRB ordering is assumed
    FastLED.addLeds<NEOPIXEL, DATA_PIN_3>(leds, 2*NUM_LEDS, NUM_LEDS); // GRB ordering is assumed
    FastLED.setBrightness(50);
    
    //re add test
    
    //**********LED screen setup**********
    for (i = 0; i < NUM_LEDS; ++i)
    {
      red[i]= CRGB::Red;
      orange[i] = CRGB::Orange;
      yellow[i] = CRGB::Yellow;
      green[i] = CRGB::Green;
      blue[i] = CRGB::Blue;
      purple[i] = CRGB::Purple;
    }

    for (i = 0; i < NUM_LEDS; ++i)
    {
        leds[i] = green[i];
        leds[NUM_LEDS+i] = green[i];
        leds[NUM_LEDS*2+i] = green[i];
    }

    FastLED.show();
    
    //**********SPI setup**********
    // Initialize the SPI communications
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV32);

    if (debug && Serial.println("SPI start"))
    {
    }

    //**********motor setup**********
    // Initialize the motor control
    motor1.init();
    motor2.init();
    motor3.init();

    if (debug && Serial.println("Motor init"))
    {
    }

    motor1.brake();
    motor2.brake();
    motor3.brake();

    if (debug && Serial.println("Motor set"))
    {
    }

    //**********controller setup**********

    //initialize the controller class
    controller.init();

    if (debug && Serial.println("Waiting for connection..."))
    {
    }

    //wait for initial controller connection
    while (!controller.connected())
    {
        delay(10);
    }

    if (debug && Serial.println("Connected..."))
    {
    }
    
    //set a deadzone for the joysticks
    controller.setJoyDeadzone(0.08);

    //**********imu initialization**********
    if (!orientation.init())
    {
        if (debug && Serial.println("Failed to initialize IMU!"))
        {
        }

        // halt program
        while (true)
        {
        }
    }

    if (debug && Serial.println("IMU start"))
    {
    }
    
    if (debug && Serial.println("Setup complete"))
    {
    }
}

//**********main loop**********
/** ***************************************************************************
* @par Main loop:
* The main loop for the arduino code,
*
 *****************************************************************************/
void loop()
{
    //**********variable setup**********

    float xp, yp;   // the instructed values for x & y movement
    float w[3];     // the rotational velocity set for each motor
    float amp = 1;  // the amplification of the rotation speed.

    float theta = 0; //the angle of the bot

    int i = 0; //generic counter var

    //row of the screen being displayed
    int screen_point = 0;
    int screen_before = 0;
    int screen_after = 0;

    CRGB * temp[3];

    //time recording var
    unsigned long time_at_controller_loss = 0;

    //Should the main code run?
    bool run_mode = false;

    //**********Stand By Code**********
      
    //wait untill 
    while(!run_mode)
    {
          
      if (controller.connected() && controller.buttonClick(START_BUTTON))
      {
          run_mode = true;
      }
      
      if (debug && Serial.println("Break out"))
      {
      }

      FastLED.clear();

      delay(10);
      
      for (i = 0; i < NUM_LEDS; ++i)
      {
          leds[i] = purple[i];
          leds[NUM_LEDS+i] = purple[i];
          leds[NUM_LEDS*2+i] = purple[i];
      }
      
      FastLED.show();
    
      motor1.brake();
      motor2.brake();
      motor3.brake();
  
      delay(10);
    
    }
    
    //**********main loop**********
    while (run_mode)
    {

        //**********imu get**********

        //update IMU estimation
        if (orientation.update())
        {
            if (debug && debug_level < 4 && Serial.print("Orientation: "))
            {
                Serial.println(orientation.Get_val());
            }
        }

        //convert the integral to radians
        theta = orientation.Get_val();

        //check if controller is connected
        if (controller.connected())
        {

            //**********controller get**********

            retrieve_controller_inputs(xp, yp, run_mode, amp);

            //**********update motor speeds**********

            calculate_motor_speed(w,amp,xp,yp,theta);

            //update time for tracking when the controller is lost
            time_at_controller_loss = micros();
        }
        else // set horizontal speed to zero if controller is not connected
        {

            //**********update for no controller**********

            //set the velocity of the motors
            w[0] = rotationSpeed;
            w[1] = rotationSpeed;
            w[2] = rotationSpeed;

            //run_mode (update based on time delay)
            if (time_at_controller_loss > (micros() - 1000000)) //one second
            {
                run_mode = false;
            }
        }

        //**********send out motor speeds**********

        if(orientation.Get_upright())
        {
          motor1.setSpeed(w[0]);
          motor2.setSpeed(w[1]);
          motor3.setSpeed(w[2]);
        }
        else
        {
          motor1.setSpeed(-w[0]);
          motor2.setSpeed(-w[1]);
          motor3.setSpeed(-w[2]);
        }

        //debug
        if (debug && debug_level < 3)
        {
            Serial.print(w[0]);
            Serial.print(',');
            Serial.print(w[1]);
            Serial.print(',');
            Serial.println(w[2]);
        }

        //**********LED's**********
        if (use_led)
        {
            
            //test if the leds need to be updated
            if (theta - screen_step > screen_step * screen_point)
            {
                //calculate the position of the other leds
                screen_before = screen_point - bot_resolution/3;
                screen_after = screen_point + bot_resolution/3;

                if (screen_before < 0)
                {
                  screen_before = screen_before + bot_resolution;
                }

                if (screen_after > bot_resolution)
                {
                  screen_after = screen_after - bot_resolution;
                }
                
//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp

                //set the temp array to red
                temp[0] = red;
                temp[1] = red;
                temp[2] = red;

                //if in a valid position, set the color of the leds
                if ((screen_before * screen_step) < (2 * PI / 10))
                {
                    temp[0] = blue; //main_screen.get_columb(screen_point);
                }
                else if ((screen_after * screen_step) < (2 * PI / 10))
                {
                    temp[1] = blue; //main_screen.get_columb(screen_point);
                }
                else if ((screen_point * screen_step) < (2 * PI / 10))
                {
                    temp[2] = blue; //main_screen.get_columb(screen_point);
                }

//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp//temp

                //update the led array to the saved array
                for (i = 0; i < NUM_LEDS; ++i)
                {
                    leds[i] = (temp[0])[i];
                    leds[i+NUM_LEDS] = (temp[1])[i];
                    leds[i+NUM_LEDS*2] = (temp[2])[i];
                }

                screen_point++;
                
                FastLED.show();
            }
            else if (screen_step * screen_point > (theta + screen_step))
            {
                screen_point = 0;
            }

            if (debug && debug_level < 3 && Serial.print("Screen point:"))
            {
                Serial.println(screen_point);
            }
        }

        delay(1);
    }
    //set motor speed to zero as the bot is no longer in run mode

}

void calculate_motor_speed(float (&w)[3],float amp,float x,float y,float theta)
{
  //radian offsets used for the first and second motors
  const float m1Offset = (-(2 * PI) / 3);
  const float m2Offset = 0;
  const float m3Offset = ((2 * PI) / 3);

  float phase = -PI/2; // the phase offset necessitated by the motor speed
  //may want the phase to be a function of the current rotation speed

  //set the target velocity of the motors
  float mag = translateSpeed * sqrt(y * y + x * x);
  float relativeAngle = atan2(y, -x) - theta;
  
  w[0] = rotationSpeed*amp + mag * sin(relativeAngle - m1Offset + phase);
  w[1] = rotationSpeed*amp + mag * sin(relativeAngle - m2Offset + phase);
  w[2] = rotationSpeed*amp + mag * sin(relativeAngle - m3Offset + phase);
            
  return;
}

void retrieve_controller_inputs(float &xp, float &yp, bool &run_mode, float &amp)
{
  static float offset = 0;

  controller.receiveData();
  
  //grab the analog value from the joystick
  
  xp = controller.joystick(DRIVE_JOYSTICK, X);
  yp = controller.joystick(DRIVE_JOYSTICK, Y);
  
  //test to see if the robot should stop
  
  if (controller.button(STOP_BUTTON))
  {
    run_mode = false;
  }
  
  //adjust the drift trim
              
  if (controller.buttonClick(PHASE_LAG))
  {
    ++orientation.gyro_to_rad;
  }
  else if (controller.buttonClick(PHASE_LEAD))
  {
    --orientation.gyro_to_rad;
  }
  
  //adjust the rotation speed
  
  if (controller.trigger(SPEED_DOWN) > 0.2) //if the trigger is sufficiently engaged.
  {
    amp = 1 - (controller.trigger(SPEED_DOWN)/2.0); //adjust the amplification variable
  }
  else if (controller.trigger(SPEED_UP) > 0.2)
  {
    amp = 1 + (controller.trigger(SPEED_UP)/2.0);
  }
  else
  {
    amp = 1;
  }
  
  //update the offset for turning
              
  if (abs(controller.joystick(TURN_JOYSTICK, X)) > 0.2)
  {
    offset = offset+(controller.joystick(TURN_JOYSTICK, X)/-120);
              
    //make sure that the offset is from 0 to 2PI
    if(offset < 0)
    {
      offset = offset + 2*PI;
    }
    else if(offset > 2*PI)
    {
      offset = offset - 2*PI;
    }
    orientation.Set_offset(offset);
  }
  
  if (debug && (debug_level < 2))
  {
    Serial.print("---------------------------------Offset: ");
    Serial.println(offset);
    Serial.print("---------------------------------X: ");
    Serial.println(xp);
    Serial.print("---------------------------------Y: ");
    Serial.println(yp);
    Serial.print("---------------------------------Amp: ");
    Serial.println(amp);
  }

  return;
}

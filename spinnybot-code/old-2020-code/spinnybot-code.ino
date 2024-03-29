/************************************************************************/ /**
*@file
*@brief The main code to load onto the NRC 2021 battle bot
*
*TO DO
* impliment 12v sense and satus led
* debug
***************************************************************************/

#include <math.h>
#include <FastLED.h>
#include <SPI.h>
#include "Nidec24hController.h" // https://github.com/sdsmt-robotics/MotorSpeedController/tree/master/Software/controllerInterfaceLib
#include "Controller.h"         // https://github.com/sdsmt-robotics/Controller/tree/master/Code/receive
#include "Imu.h"
#include "LedStrip.h" 

/*********************Pin labels***********************
 * GPIO 0 SS for the slave motor 3
 * GPIO 1 
 * GPIO 2 12V sense (ADD)
 * GPIO 3 
 * GPIO 4 SS for the slave motor 2
 * GPIO 5 SS for the slave motor 1
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
 * GPIO 16 Tx0 for serial to the xBee REQ
 * GPIO 17 Rx0 for serial to the xBee REQ
 * GPIO 18 SCK SPI to the motor boards REQ
 * GPIO 19 MISO SPI to the motor boards REQ
 * GPIO 20
 * GPIO 21
 * GPIO 22
 * GPIO 23 MOSI SPI to the motor boards REQ
 * GPIO 24
 * GPIO 25 for the led strips 3
 * GPIO 26 for the led strips 1
 * GPIO 27 for the led strips 2
 * GPIO 28
 * GPIO 29
 * GPIO 30
 * GPIO 31
 * GPIO 32
 * GPIO 33 status LED (ADD)
 * GPIO 34
 * GPIO 35
 * GPIO 36 
 * GPIO 37
 * GPIO 38
 * GPIO 39
 ***************************************************/

//**********constants for the motors**********
const float rotation = 400; // Chassis rotation speed in RPM
const float translation = 2.5; // Driving feet per sec 
const float chassisRad = 5.25; // Chassis radius in inches
const float wheelRad = 1.18; // Wheel radius in inches
const float translateSpeed = translation / wheelRad * (12 * 60 / 2 / PI);
const float rotationSpeed = rotation * chassisRad / wheelRad;
const float fullPower = 1000; // Power to run at when going full speed

//**********motor objects**********

// Pin for motor slaves
#define MOTOR_PIN_1 5
#define MOTOR_PIN_2 4
#define MOTOR_PIN_3 0

// Pins for data to led strip
#define DATA_PIN_1 25
#define DATA_PIN_2 26
#define DATA_PIN_3 27

#define IMU_PIN  22

// Motor Controller class
Nidec24hController motor1(SPI, MOTOR_PIN_1);
Nidec24hController motor2(SPI, MOTOR_PIN_2);
Nidec24hController motor3(SPI, MOTOR_PIN_3);


//**********IMU*************************
const float IMU_OFFSET = PI/3; // Angle IMU is facing relative to front of robot
Imu orientation(IMU_PIN);

//**********Controller object and key binding**********

//Create the communications object. Use Serial for the communications.
Controller controller(Serial2);

#define START_BUTTON UP //button
#define STOP_BUTTON DOWN //button
#define DRIVE_JOYSTICK LEFT //joystick
#define TURN_JOYSTICK RIGHT //joystick
#define PHASE_LEAD  RIGHT //button
#define PHASE_LAG   LEFT //button
#define SPEED_UP    RIGHT //trigger
#define SPEED_DOWN  LEFT //trigger
#define FULL_SEND   RIGHT //trigger

//**********constants for the LEDs**********

//the number of adressible horizontal stripes of leds around the robot
const int bot_resolution = 50;

// How many leds in your strip?
#define NUM_LEDS 8

// Define LED Strip angles
const float stripOffset = 0.0;  // Offset for the #1 LED Strip
float stripAngles[] = {0.0 + stripOffset,
                       2*PI/3.0 + stripOffset,
                       4*PI/3.0 + stripOffset};

// Led strips setting managers
LedStrip ledStrip1;
LedStrip ledStrip2;
LedStrip ledStrip3;

// Temps for calculating strip colors
CRGB stripColors[] = {CRGB::Red, CRGB::Red, CRGB::Red};

//the number of addressable radial led's per horizontal stripe
//CRGB leds[NUM_LEDS*3];

//const float screen_step = (2 * PI) / float(bot_resolution);

const float stripEpsilon = (2 * PI) / 60.0;

//construct screens(class is a work in progress)
//screen main_screen(bot_resolution, NUM_LEDS, bot_resolution);

//**********DATA for the LEDs********** (continued in setup)
//these arrays contain all data for static led patterns and simple 
//animations not defined by a helper function

/*CRGB red[NUM_LEDS];
CRGB orange[NUM_LEDS];
CRGB yellow[NUM_LEDS];
CRGB green[NUM_LEDS];
CRGB blue[NUM_LEDS];
CRGB purple[NUM_LEDS];*/

//**********Other global vars and consts**********

const bool debug = false;
const bool debug_orientation = false;
const bool debug_motor_speeds = false;
const int debug_level = 4; //0 to 4
const bool use_led = true;

bool fullSend = false;  // Toggle to full power mode
bool flipped = false;


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

    delay(5000);

    // start serial if debug is true
    if (debug)
    {
        //wait for serial to connect
        Serial.begin(115200);
        while (!Serial && debug)
        {
        }
    }

    if (debug)
    {
        Serial.println("Serial start");
    }

    //**********LED screen setup**********
    // Offset angles for spinning
    for (int i = 0; i < 3; i++) {
      stripAngles[i] = normalizeAngle(stripAngles[i]);
    }

    // Fast LED stuff
    FastLED.addLeds<NEOPIXEL, DATA_PIN_1>(ledStrip1.getPixelData(), NUM_LEDS); // GRB ordering is assumed
    FastLED.addLeds<NEOPIXEL, DATA_PIN_2>(ledStrip2.getPixelData(), NUM_LEDS); // GRB ordering is assumed
    FastLED.addLeds<NEOPIXEL, DATA_PIN_3>(ledStrip3.getPixelData(), NUM_LEDS); // GRB ordering is assumed
    FastLED.setBrightness(50);

    // Show initial color
    ledStrip1.fillColor(CRGB::Yellow);
    ledStrip2.fillColor(CRGB::Yellow);
    ledStrip3.fillColor(CRGB::Yellow);
    FastLED.show();
    
    //re add test
    
    
    //**********SPI setup**********
    // Initialize the SPI communications
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV32);

    if (debug && Serial.println("SPI start"))
    {
    }

    //**********motor setup**********
    if (debug && Serial.println("Motor init...")) { }
    // Initialize the motor control
    motor1.init();
    motor2.init();
    motor3.init();

    motor1.brake();
    motor2.brake();
    motor3.brake();

    if (debug && Serial.println("Motor init done.")) { }
    
    //**********imu initialization**********
    if (debug && Serial.println("Initializing IMU...")) { }
    while (!orientation.init())
    {
        if (debug && Serial.println("Failed to initialize IMU!")) { }
        delay(1000);
    }
    orientation.setOffset(IMU_OFFSET);

    if (debug && Serial.println("IMU init complete.")) { }

    ledStrip1.fillColor(CRGB::Green);
    ledStrip2.fillColor(CRGB::Green);
    ledStrip3.fillColor(CRGB::Green);
    FastLED.show();

    //**********controller setup**********
    if (debug && Serial.println("Initializing controller...")) { }
    //initialize the controller class
    controller.init();

    if (debug && Serial.println("Waiting for connection..."))
    {
    }

    //wait for initial controller connection
    while (!controller.connected())
    {
        FastLED.show();
        delay(10);
    }

    if (debug && Serial.println("Connected.")) { }
    
    //set a deadzone for the joysticks
    controller.setJoyDeadzone(0.08);

    if (debug && Serial.println("Controller init complete.")) { }
    
    if (debug && Serial.println("Setup complete!")) { }
}


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

    //time recording var
    unsigned long time_at_controller_loss = 0;
    bool controllerBeingLost = false;

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
      orientation.update();
      if (debug && Serial.println(orientation.getAngle(), 2))
      {
      }
      
      if (controller.connected())
      {
        //**********controller get**********
        driveBasic();    
      } else {
        motor1.brake();
        motor2.brake();
        motor3.brake();
      }

      // TODO: do we need this?
      //FastLED.clear();
      //delay(10);
      
      ledStrip1.fillColor(CRGB::Purple);
      ledStrip2.fillColor(CRGB::Purple);
      ledStrip3.fillColor(CRGB::Purple);
      FastLED.show();
    
  
      delay(3);
    
    }
    
    //**********main loop**********
    while (run_mode)  // Entier loop takes 2.5 to 3 ms
    {
        //**********imu get**********
        //update IMU estimation
        if (!fullSend) {
          if (orientation.update())
          {
              if ((debug && debug_level < 4) || debug_orientation)
              {
                  Serial.println(orientation.getAngle());
              }
          }
        }
        

        //convert the integral to radians
        theta = orientation.getAngle();

        //check if controller is connected
        if (controller.connected())
        {

            //**********controller get**********

            retrieve_controller_inputs(xp, yp, run_mode, amp);

            //**********update motor speeds**********

            calculate_motor_speed(w,amp,xp,yp,theta);

            //update time for tracking when the controller is lost
            time_at_controller_loss = millis();
            controllerBeingLost = false;
        }
        else // set horizontal speed to zero if controller is not connected
        {

            //**********update for no controller**********

            //set the velocity of the motors
            w[0] = rotationSpeed;
            w[1] = rotationSpeed;
            w[2] = rotationSpeed;

            //run_mode (update based on time delay)
            if (millis() - time_at_controller_loss > 2000) //one second
            {
                run_mode = false;
            }
            controllerBeingLost = true;
        }

        //**********send out motor speeds**********

        if(fullSend)
        {
          motor1.setPower(w[0]);
          motor2.setPower(w[1]);
          motor3.setPower(w[2]);
        } else {
          motor1.setSpeed(w[0]);
          motor2.setSpeed(w[1]);
          motor3.setSpeed(w[2]);
        }

        //debug
        if ((debug && debug_level < 3) || debug_motor_speeds)
        {
            Serial.print(w[0]);
            Serial.print(',');
            Serial.print(w[1]);
            Serial.print(',');
            Serial.println(w[2]);
        }

        //**********LED's**********
        // This update takes about 1ms
        if (use_led)
        {
          for (int i = 0; i < 3; i++) {
            if (!fullSend && stripAtAngle(stripAngles[i], theta, stripEpsilon)) {
              stripColors[i] = CRGB::Blue;
            } else if (controllerBeingLost) {
              stripColors[i] = CRGB::Yellow;
            } else {
              stripColors[i] = CRGB::Red;
            }
          }
  
          // Set the Colors
          ledStrip1.fillColor(stripColors[0]);
          ledStrip2.fillColor(stripColors[1]);
          ledStrip3.fillColor(stripColors[2]);
          FastLED.show();
        }
    }
    //set motor speed to zero as the bot is no longer in run mode

}

void calculate_motor_speed(float (&w)[3],float amp,float x,float y,float theta)
{
  //radian offsets used for the first and second motors
  const float m1Offset = 0;
  const float m2Offset = 2*PI/3;
  const float m3Offset = -2*PI/3;
  float phase = 0.0; // the phase offset necessitated by the motor speed
  //may want the phase to be a function of the current rotation speed

  if (!fullSend) { // Not full send mode
    //set the target velocity of the motors
    float mag = translateSpeed * sqrt(y * y + x * x);
    float relativeAngle = atan2(y, -x) - theta;
    
    w[0] = rotationSpeed*amp - mag * sin(relativeAngle - m1Offset + phase);
    w[1] = rotationSpeed*amp - mag * sin(relativeAngle - m2Offset + phase);
    w[2] = rotationSpeed*amp - mag * sin(relativeAngle - m3Offset + phase);
  } else {
    // Power control mode
    w[0] = fullPower;
    w[1] = fullPower;
    w[2] = fullPower;
  }

  // Invert if upside down
  if(flipped) {
    w[0]  = -w[0];
    w[1]  = -w[1];
    w[2]  = -w[2];
  }
}

void driveBasic() {
  const float TRANS_SPEED = 300;
  const float ROT_SPEED = 80;
  float x = controller.joystick(DRIVE_JOYSTICK, X);
  float y = -controller.joystick(DRIVE_JOYSTICK, Y);
  float r = controller.joystick(RIGHT, X);

  r = (abs(r) > 0.4 ? r : 0.0);

  float inv = (flipped ? -1.0 : 1.0);

  motor1.setSpeed(TRANS_SPEED*x * inv + ROT_SPEED * r);
  motor2.setSpeed(TRANS_SPEED*(-1/2.0 * x * inv - sqrt(3.0)/2.0 * y) + ROT_SPEED * r);
  motor3.setSpeed(TRANS_SPEED*(-1/2.0 * x * inv + sqrt(3.0)/2.0 * y) + ROT_SPEED * r);
  
}

void retrieve_controller_inputs(float &xp, float &yp, bool &run_mode, float &amp)
{
  static float offset = 0;
  bool fullSendCur = false;

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
              
  if (controller.dpadClick(PHASE_LAG))
  {
    orientation.adjustGyroConversion(1.0/360);
    Serial.println("LAG");
  }
  else if (controller.dpadClick(PHASE_LEAD))
  {
    orientation.adjustGyroConversion(-1.0/360);
    Serial.println("LEAD");
  }

  if (controller.buttonClick(LEFT))
  {
    flipped = !flipped;
    Serial.println("FLIP");
  }
  
  //adjust the rotation speed


  fullSendCur = controller.trigger(FULL_SEND) > 0.2;
  if (!fullSendCur && fullSend) {
    // Do a reset when done going full speed
    orientation.reset();
  }
  fullSend = fullSendCur;
  /*if (controller.trigger(SPEED_DOWN) > 0.2) //if the trigger is sufficiently engaged.
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
  }*/
  
  //update the offset for turning
              
  if (abs(controller.joystick(TURN_JOYSTICK, X)) > 0.2)
  {
    offset = offset+(controller.joystick(TURN_JOYSTICK, X)/-120);

    orientation.setOffset(offset);
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

bool stripAtAngle(const float &stripAngle, const float &botAngle, const float &stripEpsilon) {
  float diff = stripAngle + botAngle;  // Sum should be zero

  // Normalize
  while (diff < PI) {
    diff += 2*PI;
  }
  while (diff > PI) {
    diff -= 2*PI;
  }

  return fabs(diff) < stripEpsilon;
}

float normalizeAngle(float angle) {
  while (angle < 0) {
    angle += 2*PI;
  }
  while (angle > 2*PI) {
    angle -= 2*PI;
  }

  return angle;
}

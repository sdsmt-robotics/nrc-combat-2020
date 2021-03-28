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
 * 2
 * 3
 * 4 for the led strips
 * 5
 * 6 for the slave motor boards
 * 7 for the slave motor boards
 * 8
 * 9
 * 10 for the slave motor boards REQ
 * 11 for SPI to the motor boards REQ
 * 12 for SPI to the motor boards REQ
 * 13 for SPI to the motor boards REQ
 * A1
 * A2
 * A3
 * A4
 * A5
 * A6
 * A7
 ***************************************************/

//**********constants for the motors**********

//radian offsets used for the first and second motors
const float m1Offset = (-(2 * PI) / 3);
const float m2Offset = 0;
const float m3Offset = ((2 * PI) / 3);

//constant rotational speed for the bot
float rotation = 300; // Chassis rotation speed in RPM
float translation = 1.5; // Driving feet per sec
float chassisRad = 4; // Chassis radius in inches
float wheelRad = 1; // Wheel radius in inches
float translateSpeed = translation / wheelRad * (12 * 60 / 2 / PI);
float rotationSpeed = rotation * chassisRad / wheelRad;

//**********motor objects**********

// Pin for motor slaves
#define MOTOR_PIN_1 SS
#define MOTOR_PIN_2 6
#define MOTOR_PIN_3 7

// Motor Controller class (check that i am using this right
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
#define SPEED_UP RIGHT //triger
#define SPEED_DOWN LEFT //triger

//**********constants for the LEDs**********

//the number of adressible horizontal stripes of leds around the robot
const int bot_resolution = 50;

// How many leds in your strip?
#define NUM_LEDS 8

// Pin for data to led strip
#define DATA_PIN 4

//the number of addressable radial led's per horizontal stripe
CRGB leds[NUM_LEDS];

const float screen_step = (2 * PI) / float(bot_resolution);

//construct screens(class is a work in progress)
//screen main_screen(bot_resolution, NUM_LEDS, bot_resolution);

//**********DATA for the LEDs********** (continued in setup)
//these arrays contain all data for static led patterns and simple animations not defined by a helper function

CRGB blue[NUM_LEDS];
CRGB red[NUM_LEDS];
CRGB green[NUM_LEDS];
CRGB yellow[NUM_LEDS];

//**********Other global vars and consts**********

const bool debug = false;
const int debug_level = 3; //0 to 4
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

    //**********Testing classes and LED's**********

    //add leds for the fast led library
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); // GRB ordering is assumed

    //re add test
    
    //**********LED screen setup**********
    for (i = 0; i < NUM_LEDS; ++i)
    {
        blue[i] = CRGB::Blue;
        red[i] = CRGB::Red;
        green[i] = CRGB::Green;
        blue[i] = CRGB::Blue;
    }

    if (debug && Serial.println("Setup compleat"))
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

    float xp, yp;              // the instructed values for x & y movement
    float w1s, w2s, w3s;       // the rotational velocity set for each motor
    float amp;                 // the amplification of the rotation speed.

    float theta = 0; //the angle of the bot
    float phase = -PI/2; // the phase offset necessitated by the motor speed
    float offset = 0;
    //(may what to make this a function of the rotation speed later)

    int i = 0; //generic counter var

    int screen_point = 0; //row of the screen being displayed

    CRGB *temp;

    //time recording var
    unsigned long time_at_controller_loss = 0;

    //Should the main code run?
    bool run_mode = false;

    if (controller.connected() && controller.buttonClick(START_BUTTON))
    {
        run_mode = true;
    }

    //**********main loop**********
    while (run_mode)
    {
        
        //record the start time at the beginning of each loop
        controller.receiveData();

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
            
            if (abs(controller.joystick(TURN_JOYSTICK, X)) > 0.2) {
              offset = offset+(controller.joystick(TURN_JOYSTICK, X)/-120);
            }

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

            //**********update motor speeds**********

            //set the target velocity of the motors
            float targetAngle = atan2(yp, -xp); // X is inverted??
            float mag = translateSpeed * sqrt(yp * yp + xp * xp);
            float relativeAngle = targetAngle - theta;

            w1s = rotationSpeed*amp + mag * sin(relativeAngle - m1Offset + phase);
            w2s = rotationSpeed*amp + mag * sin(relativeAngle - m2Offset + phase);
            w3s = rotationSpeed*amp + mag * sin(relativeAngle - m3Offset + phase);

            //update time for tracking when the controller is lost
            time_at_controller_loss = micros();
        }
        else // set horizontal speed to zero if controller is not connected
        {

            //**********update motor speeds for no controller**********

            //set the velocity of the motors
            w1s = rotationSpeed;
            w2s = rotationSpeed;
            w3s = rotationSpeed;

            //run_mode (update based on time delay)
            if (time_at_controller_loss > (micros() - 1000000)) //one second
            {
                run_mode = false;
            }
        }

        //**********send out motor speeds**********

        motor1.setSpeed(w1s);
        motor2.setSpeed(w2s);
        motor3.setSpeed(w3s);

        //debug
        if (debug && debug_level < 3)
        {
            Serial.print(w1s);
            Serial.print(',');
            Serial.print(w2s);
            Serial.print(',');
            Serial.println(w3s);
        }

        //**********LED's**********
        if (use_led)
        {
            if (theta - screen_step > screen_step * screen_point)
            {
                FastLED.clear();

                //temp//temp//temp//temp//temp//temp//temp//temp
                if (theta < (2 * PI / 10))
                {
                    temp = blue; //main_screen.get_columb(screen_point);
                }
                else
                {
                    temp = red;
                }
                //temp//temp//temp//temp//temp//temp//temp//temp

                for (i = 0; i < NUM_LEDS; ++i)
                {
                    leds[i] = temp[i];
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


    if (debug && Serial.println("Break out"))
    {
    }
    
    motor1.brake();
    motor2.brake();
    motor3.brake();
}

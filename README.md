# nrc-combat-2020
Robot code for the NRC Combat competition bot. Information about this bot can be found here: http://robotics.sdsmt.edu/projects/spinnybot  

# Hardware Info
This code is meant to run on an ESP32. It uses an ICM20649 Wide-Range 6-DoF IMU for orientation sensing. It also controls three NeoPixel 8x5050 RGB LED Sticks. For motor control, it interfaces with three Nidec24h motors controlled by our custom motor controller boards. For remote control, the bot receives commands from our custom remote controls over an xBee. The full system diagram is shown below.  

![Control system diagram](http://robotics.sdsmt.edu/images/b/7/e/9/c/b7e9ccda867099f6f04a5f51d13265e70089f007-main-electrical-diagram.png)  

The main components of the control system (ESP32, xBee, voltage regulators, level shifters, etc) are held on a custom main board. The Kicad files for this are in the [spinnybot-motherboard](https://github.com/sdsmt-robotics/nrc-combat-2020/tree/master/spinnybot-motherboad) folder.  


# Software Info  
The main code for the bot is found in the [spinnybot-code](https://github.com/sdsmt-robotics/nrc-combat-2020/tree/master/spinnybot-code) folder. There are some additional arduino sketches for testing the various functionallities. These are found in the "testing" sub-folder. These tests run with the same libraries as the main control code.  

The pinouts are set up to run with the ESP32. But it should be able to run on any device with similar pin functionalities.  

## Required Libraries  
The required libraries are as follows:  
* Nidec24hController.h - Interface lib for the motor speed controller. Found [here](https://github.com/sdsmt-robotics/MotorSpeedController/tree/master/Software/controllerInterfaceLib).  
* Controller.h - Receive library for our custom remote control. Found [here](https://github.com/sdsmt-robotics/MotorSpeedController/tree/master/Software/controllerInterfaceLib).  
* FastLED.h - Code for interfacing with the LED strips. Install through the Arduino library manager.  
* Adafruit_ICM20649.h - Library for the ICM-20649 IMU. Install through the Arduino library manager.  

## Functionality  
The functionality is as follows. First, the bot will first initialize all peripherals. The LED strips are initialized first to show status throughout.Then the IMU is initialized. While this is occurring, the bot MUST remain stationary. The code runs through a calibration sequence each time, and any bumps will cause issues. During this stage the LEDs will show yellow, and will go green once complete. After, this the motors and remote control communications will be set up as well.  

For the main control loop, the bot has three main states:  
* No-spin: lights will be purple and the bot will not be spinning. Driver can drive the bot around with standard controls.  
* Spinny: bot is spinning with IMU feedback for orientation. LEDs will be red except when facing forward at which point they will be blue. Controls for driving are relative to "forward".  
* Full-spinny: lights will be red and bot will set all motors to full power. Feedback from IMU and attempts to drive will be ignored.  

/************************************
* Demo file to test the IMU tracking.
* 
* Set up ESP32, IMU, and LED strip on a spinning platform. 
* This test will light up the LED strip in the same way as 
* on the actual bot.
************************************/

#include <WiFi.h>
#include <WiFiClient.h>
//#include <WebServer.h>
#include "Imu.h" //?
#include "LedStrip.h" 
#include "index.h"  //Web page header file

// Pins for data to led strip
#define DATA_PIN 27
#define NUM_LEDS 8


Imu orientation(22);

//=======LED STuff================
// LED Strip angle stuff
const float stripOffset = 0.0;  // Offset for the #1 LED Strip
float stripAngle = 0.0 + stripOffset;

// Led strips setting manager
LedStrip ledStrip;

const float stripEpsilon = (2 * PI) / 60.0;

// Temp for figuring out strip colors
CRGB stripColor = CRGB::Red;


//==========Webserver stuff====================
//Set web server socket
//WebServer server(80);

//Enter your SSID and PASSWORD
const char* ssid = "mclaurylabs";
const char* password = "uasrobotics3d";


void setup() {
  Serial.begin(115200);

  while(!Serial) {}
  
  Serial.println("Start");
  
  //**********LED Setup Code**********
  Serial.println("Initializing LEDs...");
  // Offset angles for spinning
  stripAngle = Imu::normalizeAngle(stripAngle);

  // Fast LED stuff
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(ledStrip.getPixelData(), NUM_LEDS); // GRB ordering is assumed
  FastLED.setBrightness(50);

  // Show initial color
  ledStrip.fillColor(CRGB::Green);
  FastLED.show();

  //*******IMU Setup Code**********  
  Serial.println("Initializing IMU...");
  delay(1000);   // Give some time to let things settle for calibration
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  if(!orientation.init())
    while(true)
    {
      Serial.println("IMU setup failed");
      delay(500);
    }
    
  //*******Webserver Code**********  
  /*WiFi.mode(WIFI_STA); //Connectto your wifi
  WiFi.begin(ssid, password);

  Serial.println("Connecting to ");
  Serial.print(ssid);

  //Wait for WiFi to connect
  while(WiFi.waitForConnectResult() != WL_CONNECTED){      
      Serial.print(".");
    }
    
  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP
 
  server.on("/", handleRoot);      //This is display page
  server.on("/readADC", handleADC);//To get update of ADC Value only
 
  server.begin();                  //Start server
  Serial.println("HTTP server started");
*/
  Serial.println("Done!");
}

void loop() {
  float theta = 0; //the angle of the bot
    
  while(true)
  {
    orientation.update();
    theta = orientation.getAngle();
    Serial.print(theta, 4);
    Serial.print(", ");
    Serial.print(orientation.isUpright());
    Serial.println("");

    //**********LED's**********
    // This update takes about 1ms
    if (stripAtAngle(stripAngle, theta, stripEpsilon)) {
      stripColor = CRGB::Blue;
    } else {
      stripColor = CRGB::Red;
    }

    // Set the Colors
    ledStrip.fillColor(stripColor);
    FastLED.show();
    
    //server.handleClient();
    delay(1);
  }
  
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

void handleRoot() {
 String s = MAIN_page; //Read HTML contents
 //server.send(200, "text/html", s); //Send web page
}
 
void handleADC() {
 String adcValue = String(orientation.getRpm());
 
 //server.send(200, "text/plane", adcValue); //Send ADC value only to client ajax request
}

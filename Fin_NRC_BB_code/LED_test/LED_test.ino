#include "/home/joseph/Desktop/Robot/NRC/nrc-combat-2020/Fin_NRC_BB_code/POV_Display.cpp" //?

// How many leds in your strip?
#define NUM_LEDS 8

// Pin for data to led strip
#define DATA_PIN_1 17
#define DATA_PIN_2 18
#define DATA_PIN_3 19

//the number of addressable radial led's per horizontal stripe
CRGB leds1[NUM_LEDS];
CRGB leds2[NUM_LEDS];
CRGB leds3[NUM_LEDS];

//construct screens(class is a work in progress)
//screen  main_screen(30,NUM_LEDS,30);

CRGB red[NUM_LEDS];
CRGB orange[NUM_LEDS];
CRGB yellow[NUM_LEDS];
CRGB green[NUM_LEDS];
CRGB blue[NUM_LEDS];
CRGB purple[NUM_LEDS];

void setup() {

  Serial.begin(9600);
  while(!Serial) {}

  Serial.println("start");
  
  int i;

  //add leds for the fast led library
  FastLED.addLeds<NEOPIXEL, DATA_PIN_1>(leds1, NUM_LEDS); // GRB ordering is assumed
  FastLED.addLeds<NEOPIXEL, DATA_PIN_2>(leds2, NUM_LEDS); // GRB ordering is assumed
  FastLED.addLeds<NEOPIXEL, DATA_PIN_3>(leds3, NUM_LEDS); // GRB ordering is assumed
  FastLED.setBrightness(50);

  for (i = 0; i < NUM_LEDS; ++i)
  {
    red[i]= CRGB::Red;
    orange[i] = CRGB::Orange;
    yellow[i] = CRGB::Yellow;
    green[i] = CRGB::Green;
    blue[i] = CRGB::Blue;
    purple[i] = CRGB::Purple;
  }
  
  //main_screen.set_columb(0,blue);
  //main_screen.set_columb(2,red);
  //main_screen.set_columb(4,green);
  //main_screen.set_columb(6,yellow);
  //main_screen.set_columb(8,black);

}

void loop() {

  int i;
  int j;

  CRGB * temp[11];
  
  temp[0] = red;
  temp[1] = orange;
  temp[2] = yellow;
  temp[3] = green;
  temp[4] = blue;
  temp[5] = purple;
  temp[6] = red;
  temp[7] = orange;
  temp[8] = yellow;
  temp[9] = green;
  temp[10] = blue;
  temp[11] = purple;

  while(true)
  {
    for(i = 0; i < 9; ++i)
    {
      //temp = main_screen.get_columb(i);
  
      for(j = 0; j < NUM_LEDS; ++j)
      {
        leds1[j] = (temp[i])[j];
        leds2[j] = (temp[i+1])[j];
        leds3[j] = (temp[i+2])[j];
      } 
      
      FastLED.show();
      delay(500);
      FastLED.clear();  // clear all pixel data

      Serial.println(i);
      
    }
  
    Serial.println("test");
  }
}

#include "/home/joseph/Desktop/Robot/NRC/nrc-combat-2020/Fin_NRC_BB_code/POV_Display.cpp" //?

// How many leds in your strip?
#define NUM_LEDS 8

// Pin for data to led strip
#define DATA_PIN 4

//construct screens(class is a work in progress)
screen  main_screen(30,NUM_LEDS,30);

//the number of addressable radial led's per horizontal stripe
CRGB leds[NUM_LEDS];

CRGB blue[NUM_LEDS];
CRGB red[NUM_LEDS];
CRGB green[NUM_LEDS];
CRGB yellow[NUM_LEDS];
CRGB black[NUM_LEDS];

void setup() {

  int i;

  //add leds for the fast led library
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed

  for(i =0; i < NUM_LEDS; ++i)
  {
    blue[i] = CRGB::Blue;
    red[i] = CRGB::Red;
    green[i] = CRGB::Green;
    yellow[i] = CRGB::Yellow;
    black[i] = CRGB::Black;
  }
  
  main_screen.set_columb(0,blue);
  main_screen.set_columb(2,red);
  main_screen.set_columb(4,green);
  main_screen.set_columb(6,yellow);
  main_screen.set_columb(8,black);
  


}

void loop() {

  int i;
  int j;

  CRGB * temp;
  
  for(i = 0; i < 9; ++i)
  {
    temp = main_screen.get_columb(i);

    for(j = 0; j < NUM_LEDS; ++j)
    {
      leds[j] = temp[j];
    }
    
    FastLED.show();
    delay(1000);
    FastLED.clear();  // clear all pixel data
    
  }
  

}

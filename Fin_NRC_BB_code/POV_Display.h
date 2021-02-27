/************************************************************************/ /**
*@file
*@brief Contains the POV structures and classes prototypes.
***************************************************************************/
#ifndef _POV
#define _POV

#include "Arduino.h"
#include <FastLED.h>

struct stripe;


/***********************************************************************/ /**
*@class
*@brief This class manages the virtual screens used for animations and
*backgrounds of the display. Essentially it groups "stripe"s together to make
*images.
***************************************************************************/
class screen
{
public:
  //input the resolution in number columns of the screen as well as the total columns stored
  screen(int resolution_in, int num_leds_in, int stored_in);

  //get the pointer to the relevant stripe
  CRGB *get_columb(int c);

  //set the indicated columb
  void set_columb(int c, CRGB columb_pixel[]);

private:
  int num_leds;
  stripe *columns  = NULL;      //pointer to the array of strips
  int resolution;       //the number of strips that should be displayed at once
  int stored;           //the total number of strips
  int scan_start = 0;   //where the scan in stored starts for the desired resolution
  stripe *blank_stripe  = NULL; //the blank stripe for this screen (background)
};

/***********************************************************************/ /**
*@struct
*@brief Holds the information for a single LED row in an easy to work with
*format.
***************************************************************************/
struct stripe
{
  //default constructor
  stripe();

  //pointer to the array of CRGB leds
  CRGB *pixels = NULL;

  ~stripe();

};

#endif

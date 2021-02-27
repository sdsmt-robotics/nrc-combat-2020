/************************************************************************/ /**
*@file
*@brief Contains the POV class functions.
***************************************************************************/
#include "POV_Display.h"
 
 
/** ***************************************************************************
* @par Description:
* WIP
*
* @param [in] key : WIP
* @param [out] x : WIP
*
* @returns none.
*****************************************************************************/
screen::screen(int resolution_in, int leds, int stored_in) : num_leds(leds),
                                                            resolution(resolution_in), stored(stored_in)
{
 
   //generic counter
   int i = 0;
 
   //set up the stripe for a blank/black default
   blank_stripe = new stripe;
 
   //see if the new array is valid and not allow the class to be used if it isn't
   if (blank_stripe == NULL)
   {
       return;
   }
 
   //set up the pixels of the
   blank_stripe->pixels = new CRGB[num_leds];
 
   //see if the new array is valid and not allow the class to be used if it isn't
   if (blank_stripe->pixels == NULL)
   {
       return;
   }
 
   for (i = 0; i < num_leds; ++i)
   {
       blank_stripe->pixels[i] = CRGB::Black;
   }
 
   //make a new array of stripe structures of the total size of stored
   //data for the POOR screen layer
   stripe *temp = new stripe[stored];
 
   //see if the new array is valid and not allow the class to be used if it isn't
   if (temp == NULL)
   {
       return;
   }
 
   for (i; i < stored_in; ++i)
   {
       //initialize all of the screens to the default
       temp[i].pixels = blank_stripe->pixels;
   }
 
   //update the columns pointer
   columns = temp;
 
   return;
}
 
/** ***************************************************************************
* @par Description:
* WIP
*
* @param [in] key : WIP
* @param [out] x : WIP
*
* @returns none.
*****************************************************************************/
void screen::set_columb(int c, CRGB *new_stripe)
{
 
   //if c is valid, update the corresponding strip
   if (c < stored)
   {
       (columns[c]).pixels = new_stripe;
   }
 
   return;
}
 
/** ***************************************************************************
* @par Description:
* WIP
*
* @param [in] key : WIP
* @param [out] x : WIP
*
* @returns none.
*****************************************************************************/
CRGB *screen::get_columb(int c)
{
   //if the input is invalid, return a blank stripe
   if (c >= resolution)
   {
       return blank_stripe->pixels;
   }
 
   //return the correct stripe
   return (columns[c]).pixels;
}
 
/** ***************************************************************************
* @par Description:
* WIP
*
* @param [in] key : WIP
* @param [out] x : WIP
*
* @returns none.
*****************************************************************************/
stripe::stripe(){};
 
/** ***************************************************************************
* @par Description:
* WIP
*
* @param [in] key : WIP
* @param [out] x : WIP
*
* @returns none.
*****************************************************************************/
stripe::~stripe()
{
   if (pixels != NULL)
       delete pixels;
 
   return;
}


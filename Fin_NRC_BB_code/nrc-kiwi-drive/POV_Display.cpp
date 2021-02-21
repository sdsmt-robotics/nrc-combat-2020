/************************************************************************//**
*@file
*@brief Contains the POV class funtions.
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
stripe::stripe(int num_leds)
{
    int i = 0;

    pixels = new CRGB[num_leds]

        for (i; i < num_leds; ++i)
    {
        pixels[i] = CRGB::Black;
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
screen::screen(int resolution_in, int leds, int stored_in) : num_leds(leds), resolution(resolution_in) ,stored(stored_in)
{

    //make a new array of stripe structures of the total size of stored
    stripe *temp = new stripe[stored];

    //see if the new array is valid and stop the program if it is not
    while (temp == NULL)
    {
        //Serial.println("Failed get memory for stripe array!");
        delay(2000);
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
void screen::set_columb(int c, CRGB columb_pixel[NUM_LEDS])
{
    int i = 0;

    //if c is valid, update the corresponding strip
    if (c < stored)
    {
        for (i; i < NUM_LEDS; ++i)
        {
            (columns[c]).pixels[i] = columb_pixel[i];
        }
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
CRGB* screen::get_columb(int c)
{
    //if the input is invalid, return a blank stripe
    if (c >= resolution)
    {
        return blank_stripe.pixels;
    }

    //return the correct stripe
    return (columns[c]).pixels;
}

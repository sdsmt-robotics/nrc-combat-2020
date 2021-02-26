/************************************************************************/ /**
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
screen::screen(int resolution_in, int leds, int stored_in)
{
    //save struct inputs
    num_leds = leds;
    resolution = resolution_in;
    stored = stored_in;

    int i = 0;

    blank_stripe = new stripe[stored_in];

    //see if the new array is valid and not allow the class to be used if it isent
    if (blank_stripe == nullptr)
    {
        return;
    }

    //make a new array of stripe structures of the total size of stored
    stripe *temp = new stripe[stored];

    //see if the new array is valid and not allow the class to be used if it isent
    if (temp == nullptr)
    {
        return;
    }


    for(i; i < stored_in; ++i)
    {
        if(!(temp[i].init(num_leds)))
            return;
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
void screen::set_columb(int c, CRGB columb_pixel[])
{
    int i = 0;

    //if c is valid, update the corresponding strip
    if (c < stored)
    {
        for (i; i < num_leds; ++i)
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
stripe::stripe(int num_leds)
{

    stripe::init(num_leds);

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
bool stripe::init(int num_leds)
{
    int i = 0;

    pixels = new CRGB[num_leds];

    if (pixels == nullptr)
    {
        return false;
    }

    for (i; i < num_leds; ++i)
    {
        pixels[i] = CRGB::Black;
    }

    return true;
}
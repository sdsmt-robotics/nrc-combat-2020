# Motor Speed Controller  
Code for the ATtiny motor speed controller.



# Programming the ATtiny  
To program on the ATTiny on the arduino IDE

Follow inststructions on https://www.sparkfun.com/news/2237

The data sheet for the ATTiny84a is at http://ww1.microchip.com/downloads/en/devicedoc/8183s.pdf

The pins can be a little werid with multiple pins being labeled as 0.

The pin in digitalWrite(0,HIGH) and attachInterrupt(0 , blink, RISING) are pins 13 and 5 respectively.
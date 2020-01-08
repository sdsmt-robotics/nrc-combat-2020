
# Razor IMU
This is information on setting up the Sparkfun Razor IMU.

# IMU Info
![enter image description here](https://cdn.sparkfun.com//assets/parts/3/4/5/4/09623-01b.jpg?resize=718,448)

This is the Sparkfun SEN-09623 9DOF Razor IMU. This is an old device, (circa 2009) and there are currently two newer versions available.

Sparkfun website: [https://www.sparkfun.com/products/retired/9623](https://www.sparkfun.com/products/retired/9623)

This device combines four sensors and processes data from them using an Atmega328p.  
 - LY530AL - single-axis gyro (z-axis)  
 - LPR530AL - dual-axis gyro (x&y-axis)  
 - ADXL345 - triple-axis accelerometer  
 - HMC5843 - triple-axis magnetometer  

The accelerometer and magnetometer communicate using i2c. The two gyros send values using anolog outputs with x-axis->A1, y-axis->A2, and z-axis->A0.  


# Usage
**Libraries:**
This code depends on the following libraries from [jrowberg's i2cdevlib repo](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino).  
 - I2Cdev
 - HMC5843
 - ADXL345

These can be installed by simply copying the folders from jrowberg's repo into the Arduino libraries folder (typically Documents\Arduino\libraries). 


**Programming:**

 1. Connect the IMU to the PC using a [3.3v FTDI cable](https://www.sparkfun.com/products/9873).
 2. Open the Arduino IDE.
 3. Choose the correct COM port and set board = "Arduino Pro or Pro Mini" and processor = "ATmega328P (3.3v, 8mhz)".
 4. Upload the code as normal.

**Interfacing With Another Device:**
The device communicates with other devices via serial. In the example programs the data is sent as text for debugging 
purposes and can be read using the serial monitor at 38400baud.

In the main program, data is sent in response to a request for data (ascii Enquiry control character, 0x05). Data is then sent in binary from the imu. This is read by some other device connected by serial. 

Note: the imu uses 3.3v logic. Its transmit line can be attached directly to a 5v device with no issues. The receive line however, needs either a level shifter (preferred) or a 10k resistor (janky) to be put in the line to prevent damage.
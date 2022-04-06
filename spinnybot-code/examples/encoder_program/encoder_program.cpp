/**
 * Program to flash the encoder. This reconfigures them to output in step/direction 
 * mode. This will output 180 pulses (360 equally-spaced edges) + a direciton signal.
 * 
 * The following steps **must be followed EXACTLY**. Otherwise, the encoder WILL die. 
 * 1. Run the program with the 'r' option. This will output the current setting values.
 * 2. Remove 5V from encoder.
 * 3. Put a ~10uF + ~100nF capacitor between PROG and GND (short wire between cap and connection).
 * 4. Connect a 8V-8.5V (POWERED OFF) source to the PROG pin.
 * 5. Turn 5V source on.
 * 6. Turn 8V source on.
 * 7. Run the program with the 'p' option. 
 * 8. Power off the 8V source.
 * 9. Power off the 5V source.
 * 10. Remove the capacitors and 8V source.
 * 11. Turn 5V source on.
 * 12. Run the program with the 'r' option to verify programming. 12th bit should be '1'.
 * 13. Observe the counts per rotation. Should be 360.
 */
#include <Arduino.h>
#include "AS5134.h"


// Create the encoder object
//AS5134(dioPin, csPin, clkPin)
AS5134 encoder(4, 6, 7);

void print64Bit(uint64_t val) {
    for (int i = 63; i >= 0; i--) {
        Serial.print(uint8_t((val >> i) & 1));
        if (i % 4 == 0 && i != 0) {
            Serial.print(' ');
        }
    }
    Serial.print('\n');
}

unsigned long tot = 0;
void encIsr() {
    tot++;
}

void setup() {
    Serial.begin(115200);
    encoder.init();

    //Wait for user
    while (Serial.available()) Serial.read();
    Serial.println("Enter command: \n'p' -> program, \n's' -> soft program, \n'r' -> read OTP, \n't' -> start reading ticks.");
    while (!Serial.available()) delay(1);
    char val = Serial.read();

    if (val == 'p' || val == 's') {
        // Program the encoder to run in the correct mode (pulsed output)
        Serial.println("Programming encoder...");
        // Put the encoder into OTP mode
        encoder.setData(EN_PROG, 0b1000110010101110); 

        // Read the current data from the encoder
        uint64_t data = 0;
        delay(250);
        data = encoder.getData(READ_OTP, SISO_EXTENDED_DATAWIDTH);
        Serial.print("ORIG:   \t");
        print64Bit(data);
        delay(250);

        // Send the new config
        uint8_t R_add_bit = 0b11001;
        //data |= /*(uint64_t(R_add_bit) << 16) |*/ (uint64_t(1) << 12);
        //data = (uint64_t(R_add_bit) << 16) | (uint64_t(1) << 12);
        //data = 0b0011101100011011100001001000100000110000111000000001000000000000;
        data = 0b110010001000000000000;
        Serial.print("NEW:    \t");
        print64Bit(data);
    while (Serial.available()) Serial.read();
    Serial.println("Hit go.");
    while (!Serial.available()) delay(1);
        delay(250);
        encoder.setData((val == 'p' ? PROG_OTP : WRITE_OTP), data, SISO_EXTENDED_DATAWIDTH);

        // Check the OTP OK bit
        delay(250);
        Serial.print("OTP: ");
        Serial.println((encoder.getData(RD_MT_COUNTER) >> 6) & 1);

        // Read back the data
        delay(250);
        data = encoder.getData(READ_OTP, SISO_EXTENDED_DATAWIDTH);
        Serial.print("UPDATED:\t");
        print64Bit(data);


        Serial.println("Done!");
    } else if (val == 'r') {
        // Put the encoder into OTP mode
        encoder.setData(EN_PROG, 0b1000110010101110); 

        // Read the current data from the encoder
        uint64_t data = 0;
        delay(250);
        data = encoder.getData(READ_OTP, SISO_EXTENDED_DATAWIDTH);
        Serial.println("OTP Register: ");
        print64Bit(data);
    }
    
    attachInterrupt(digitalPinToInterrupt(2), encIsr, CHANGE);
}

void loop() {
    delay(500);

    // Check for user input to reset total
    if (Serial.available()) {
        char readChar = Serial.read();
        if (readChar == ' ') {
            tot = 0;
        }
    }

    Serial.println(tot);
}

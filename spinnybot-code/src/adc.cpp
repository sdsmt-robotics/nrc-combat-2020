/*
  This is a library written for the TI ADS122C04
  24-Bit 4-Channel 2-kSPS Delta-Sigma ADC With I2C Interface

  It allows you to measure temperature very accurately using a
  Platinum Resistance Thermometer

  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!

  Written by: Paul Clark (PaulZC)
  Date: May 4th 2020

  Based on the TI datasheet:
  https://www.ti.com/product/ADS122C04
  https://www.ti.com/lit/ds/symlink/ads122c04.pdf
  Using the example code from the "High Precision Temperature Measurement
  for Heat and Cold Meters Reference Design" (TIDA-01526) for reference:
  http://www.ti.com/tool/TIDA-01526
  http://www.ti.com/lit/zip/tidcee5

  The MIT License (MIT)
  Copyright (c) 2020 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "adc.h"

SFE_ADS122C04::SFE_ADS122C04(HardwareSerial *serial) {
  // Constructor
  _i2cPort = serial;
}

// Attempt communication with the device and initialise it
// Return true if successful
bool SFE_ADS122C04::begin(int baud) {
  _i2cPort->begin(baud);
  _wireMode = ADS122C04_RAW_MODE; // Default to using 'safe' settings (disable
                                  // the IDAC current sources)

  delay(1); // wait for power-on reset to complete (datasheet says we should do
            // this)

  reset(); // reset the ADS122C04 (datasheet says we should do this)

  return (configureADCmode(
      ADS122C04_RAW_MODE)); // Default to using 'safe' settings (disable the
                            // IDAC current sources)
}

// Configure the chip for the selected wire mode
boolean SFE_ADS122C04::configureADCmode(uint8_t wire_mode, uint8_t rate) {
  ADS122C04_initParam initParams; // Storage for the chip parameters

  if (wire_mode == ADS122C04_4WIRE_MODE) // 4-wire mode
  {
    initParams.inputMux =
        ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_8; // Set the gain to 8
    initParams.pgaBypass =
        ADS122C04_PGA_ENABLED; // The PGA must be enabled for gains >= 8
    initParams.dataRate =
        rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode =
        ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref =
        ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn =
        ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn =
        ADS122C04_DCNT_DISABLE;                    // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn =
        ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent =
        ADS122C04_IDAC_CURRENT_1000_UA;           // Set the IDAC current to 1mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3; // Route IDAC1 to AIN3
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_4WIRE_MODE;                 // Update the wire mode
  } else if (wire_mode == ADS122C04_4WIRE_HI_TEMP)    // 4-wire mode for high
                                                      // temperatures (gain = 4)
  {
    initParams.inputMux =
        ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_4;      // Set the gain to 4
    initParams.pgaBypass = ADS122C04_PGA_ENABLED; // Enable the PGA
    initParams.dataRate =
        rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode =
        ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref =
        ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn =
        ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn =
        ADS122C04_DCNT_DISABLE;                    // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn =
        ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent =
        ADS122C04_IDAC_CURRENT_1000_UA;           // Set the IDAC current to 1mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3; // Route IDAC1 to AIN3
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_4WIRE_HI_TEMP;              // Update the wire mode
  } else if (wire_mode == ADS122C04_3WIRE_MODE)       // 3-wire mode
  {
    initParams.inputMux =
        ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_8; // Set the gain to 8
    initParams.pgaBypass =
        ADS122C04_PGA_ENABLED; // The PGA must be enabled for gains >= 8
    initParams.dataRate =
        rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode =
        ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref =
        ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn =
        ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn =
        ADS122C04_DCNT_DISABLE;                    // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn =
        ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent =
        ADS122C04_IDAC_CURRENT_500_UA; // Set the IDAC current to 0.5mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN2;  // Route IDAC1 to AIN2
    initParams.routeIDAC2 = ADS122C04_IDAC2_AIN3;  // Route IDAC2 to AIN3
    _wireMode = ADS122C04_3WIRE_MODE;              // Update the wire mode
  } else if (wire_mode == ADS122C04_3WIRE_HI_TEMP) // 3-wire mode for high
                                                   // temperatures (gain = 4)
  {
    initParams.inputMux =
        ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_4;      // Set the gain to 4
    initParams.pgaBypass = ADS122C04_PGA_ENABLED; // Enable the PGA
    initParams.dataRate =
        rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode =
        ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref =
        ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn =
        ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn =
        ADS122C04_DCNT_DISABLE;                    // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn =
        ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent =
        ADS122C04_IDAC_CURRENT_500_UA; // Set the IDAC current to 0.5mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN2; // Route IDAC1 to AIN2
    initParams.routeIDAC2 = ADS122C04_IDAC2_AIN3; // Route IDAC2 to AIN3
    _wireMode = ADS122C04_3WIRE_HI_TEMP;          // Update the wire mode
  } else if (wire_mode == ADS122C04_2WIRE_MODE)   // 2-wire mode
  {
    initParams.inputMux =
        ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_8; // Set the gain to 8
    initParams.pgaBypass =
        ADS122C04_PGA_ENABLED; // The PGA must be enabled for gains >= 8
    initParams.dataRate =
        rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode =
        ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref =
        ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn =
        ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn =
        ADS122C04_DCNT_DISABLE;                    // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn =
        ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent =
        ADS122C04_IDAC_CURRENT_1000_UA;           // Set the IDAC current to 1mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3; // Route IDAC1 to AIN3
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_2WIRE_MODE;                 // Update the wire mode
  } else if (wire_mode == ADS122C04_2WIRE_HI_TEMP)    // 2-wire mode for high
                                                      // temperatures (gain = 4)
  {
    initParams.inputMux =
        ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_4;      // Set the gain to 4
    initParams.pgaBypass = ADS122C04_PGA_ENABLED; // Enable the PGA
    initParams.dataRate =
        rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode =
        ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref =
        ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn =
        ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn =
        ADS122C04_DCNT_DISABLE;                    // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn =
        ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent =
        ADS122C04_IDAC_CURRENT_1000_UA;           // Set the IDAC current to 1mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3; // Route IDAC1 to AIN3
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_2WIRE_HI_TEMP;              // Update the wire mode
  } else if (wire_mode ==
             ADS122C04_TEMPERATURE_MODE) // Internal temperature mode
  {
    initParams.inputMux =
        ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_1; // Set the gain to 1
    initParams.pgaBypass = ADS122C04_PGA_DISABLED;
    initParams.dataRate =
        rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode =
        ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref =
        ADS122C04_VREF_INTERNAL; // Use the internal 2.048V reference
    initParams.tempSensorEn =
        ADS122C04_TEMP_SENSOR_ON; // Enable the temperature sensor
    initParams.dataCounterEn =
        ADS122C04_DCNT_DISABLE;                    // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn =
        ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent =
        ADS122C04_IDAC_CURRENT_OFF; // Disable the IDAC current
    initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED; // Disable IDAC1
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_TEMPERATURE_MODE;           // Update the wire mode
  } else if (wire_mode == ADS122C04_RAW_MODE) // Raw mode : disable the IDAC and
                                              // use the internal reference
  {
    initParams.inputMux =
        ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_1; // Set the gain to 1
    initParams.pgaBypass = ADS122C04_PGA_DISABLED;
    initParams.dataRate =
        rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode =
        ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref =
        ADS122C04_VREF_INTERNAL; // Use the internal 2.048V reference
    initParams.tempSensorEn =
        ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn =
        ADS122C04_DCNT_DISABLE;                    // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn =
        ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent =
        ADS122C04_IDAC_CURRENT_OFF; // Disable the IDAC current
    initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED; // Disable IDAC1
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_RAW_MODE;                   // Update the wire mode
  } else {

    return (false);
  }
  return (ADS122C04_init(&initParams)); // Configure the chip
}

float SFE_ADS122C04::readPT100Centigrade(
    void) // Read the temperature in Centigrade
{
  raw_voltage_union raw_v; // union to convert uint32_t to int32_t
  unsigned long start_time =
      millis();         // Record the start time so we can timeout
  boolean drdy = false; // DRDY (1 == new data is ready)
  float ret_val = 0.0;  // Return value
  float RTD, POLY;      // Variables needed to convert RTD to Centigrade

  // Start the conversion (assumes we are using single shot mode)
  start();

  // Wait for DRDY to go valid
  while ((drdy == false) &&
         (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT))) {
    delay(5); // Don't pound the bus too hard
    drdy = checkDataReady();
  }

  // Check if we timed out
  if (drdy == false) {
    return (ret_val);
  }

  // Read the conversion result
  if (ADS122C04_getConversionData(&raw_v.UINT32) == false) {

    return (ret_val);
  }

  // The raw voltage is in the bottom 24 bits of raw_temp
  // If we just do a <<8 we will multiply the result by 256
  // Instead pad out the MSB with the MS bit of the 24 bits
  // to preserve the two's complement
  if ((raw_v.UINT32 & 0x00800000) == 0x00800000) {
    raw_v.UINT32 |= 0xFF000000;
  }

  // raw_v.UINT32 now contains the ADC result, correctly signed
  // Now we need to convert it to temperature using the PT100 resistance,
  // the gain, excitation current and reference resistor value

  // Formulae are taken from:
  // http://www.ti.com/lit/an/sbaa275/sbaa275.pdf
  // https://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf

  // 2^23 is 8388608
  RTD = ((float)raw_v.INT32) / 8388608.0; // Load RTD with the scaled ADC value
  RTD *= PT100_REFERENCE_RESISTOR;        // Multiply by the reference resistor
  // Use the correct gain for high and low temperatures
  if ((_wireMode == ADS122C04_4WIRE_HI_TEMP) ||
      (_wireMode == ADS122C04_3WIRE_HI_TEMP) ||
      (_wireMode == ADS122C04_2WIRE_HI_TEMP)) {
    RTD /= PT100_AMP_GAIN_HI_TEMP; // Divide by the amplifier gain for high
                                   // temperatures
  } else {
    RTD /= PT100_AMPLIFIER_GAIN; // Divide by the amplifier gain for low
                                 // temperatures
  }
  if ((_wireMode == ADS122C04_3WIRE_MODE) ||
      (_wireMode == ADS122C04_3WIRE_HI_TEMP)) // If we are using 3-wire mode
  {
    RTD *= 2.0; // 3-wire mode needs a factor of 2
  }

  // RTD now contains the PT100 resistance in Ohms
  // Now we need to convert this to temperature

  // Calculate the temperature
  ret_val = RTD * -23.10e-9;
  ret_val += 17.5848089e-6;
  ret_val = sqrt(ret_val);
  ret_val -= 3.9083e-3;
  ret_val /= -1.155e-6;

  //  Check if the temperature is positive, return if it is
  if (ret_val >= 0.0)
    return (ret_val);

  // The temperature is negative so we need to use a different formula
  ret_val = -242.02;
  ret_val += 2.2228 * RTD;
  POLY = RTD * RTD; // Load the polynomial with RTD^2
  ret_val += 2.5859e-3 * POLY;
  POLY *= RTD; // Load the polynomial with RTD^3
  ret_val -= 4.8260e-6 * POLY;
  POLY *= RTD; // Load the polynomial with RTD^4
  ret_val -= 2.8183e-8 * POLY;
  POLY *= RTD; // Load the polynomial with RTD^5
  ret_val += 1.5243e-10 * POLY;

  return (ret_val);
}

float SFE_ADS122C04::readPT100Fahrenheit(
    void) // Read the temperature in Fahrenheit
{
  return ((readPT100Centigrade() * 1.8) +
          32.0); // Read Centigrade and convert to Fahrenheit
}

// Read the raw signed 24-bit ADC value as int32_t
// The result needs to be multiplied by VREF / GAIN to convert to Volts
int32_t SFE_ADS122C04::readRawVoltage(uint8_t rate) {
  raw_voltage_union raw_v; // union to convert uint32_t to int32_t
  unsigned long start_time =
      millis();         // Record the start time so we can timeout
  boolean drdy = false; // DRDY (1 == new data is ready)
  uint8_t previousWireMode =
      _wireMode; // Record the previous wire mode so we can restore it
  uint8_t previousRate =
      ADS122C04_Reg.reg1.bit
          .DR; // Record the previous rate so we can restore it

  // Configure the ADS122C04 for raw mode
  // Disable the IDAC, use the internal 2.048V reference and set the gain to 1
  if ((configureADCmode(ADS122C04_RAW_MODE, rate)) == false) {

    return (0);
  }

  // Start the conversion (assumes we are using single shot mode)
  start();

  // Wait for DRDY to go valid
  while ((drdy == false) &&
         (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT))) {
    delay(5); // Don't pound the bus too hard
    drdy = checkDataReady();
  }

  // Check if we timed out
  if (drdy == false) {

    configureADCmode(previousWireMode,
                     previousRate); // Attempt to restore the previous wire mode
    return (0);
  }

  // Read the conversion result
  if (ADS122C04_getConversionData(&raw_v.UINT32) == false) {

    configureADCmode(previousWireMode,
                     previousRate); // Attempt to restore the previous wire mode
    return (0);
  }

  // Restore the previous wire mode
  if ((configureADCmode(previousWireMode, previousRate)) == false) {

    return (0);
  }

  // The raw voltage is in the bottom 24 bits of raw_temp
  // If we just do a <<8 we will multiply the result by 256
  // Instead pad out the MSB with the MS bit of the 24 bits
  // to preserve the two's complement
  if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
    raw_v.UINT32 |= 0xFF000000;
  return (raw_v.INT32);
}

// Read the raw signed 24-bit ADC value as uint32_t
// The ADC data is returned in the least-significant 24-bits
// Higher functions will need to convert the result to (e.g.) int32_t
uint32_t SFE_ADS122C04::readADC(void) {
  uint32_t ret_val; // The return value

  // Read the conversion result
  if (ADS122C04_getConversionData(&ret_val) == false) {

    return (0);
  }

  return (ret_val);
}

// Read the internal temperature
float SFE_ADS122C04::readInternalTemperature(uint8_t rate) {
  internal_temperature_union int_temp; // union to convert uint16_t to int16_t
  uint32_t raw_temp;                   // The raw temperature from the ADC
  unsigned long start_time =
      millis();         // Record the start time so we can timeout
  boolean drdy = false; // DRDY (1 == new data is ready)
  float ret_val = 0.0;  // The return value
  uint8_t previousWireMode =
      _wireMode; // Record the previous wire mode so we can restore it
  uint8_t previousRate =
      ADS122C04_Reg.reg1.bit
          .DR; // Record the previous rate so we can restore it

  // Enable the internal temperature sensor
  // Reading the ADC value will return the temperature
  if ((configureADCmode(ADS122C04_TEMPERATURE_MODE, rate)) == false) {

    return (ret_val);
  }

  // Start the conversion
  start();

  // Wait for DRDY to go valid
  while ((drdy == false) &&
         (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT))) {
    delay(5); // Don't pound the bus too hard
    drdy = checkDataReady();
  }

  // Check if we timed out
  if (drdy == false) {

    configureADCmode(previousWireMode,
                     previousRate); // Attempt to restore the previous wire mode
    return (ret_val);
  }

  // Read the conversion result
  if (ADS122C04_getConversionData(&raw_temp) == false) {

    configureADCmode(previousWireMode,
                     previousRate); // Attempt to restore the previous wire mode
    return (ret_val);
  }

  // Restore the previous wire mode
  if ((configureADCmode(previousWireMode, previousRate)) == false) {

    return (ret_val);
  }

  // The temperature is in the top 14 bits of the bottom 24 bits of raw_temp
  int_temp.UINT16 = (uint16_t)(raw_temp >> 10); // Extract the 14-bit value

  // The signed temperature is now in the bottom 14 bits of int_temp.UINT16
  // If we just do a <<2 we will multiply the result by 4
  // Instead we will pad out the two MS bits with the MS bit of the 14 bits
  // to preserve the two's complement
  if ((int_temp.UINT16 & 0x2000) == 0x2000) // Check if the MS bit is 1
  {
    int_temp.UINT16 |= 0xC000; // Value is negative so pad with 1's
  } else {
    int_temp.UINT16 &=
        0x3FFF; // Value is positive so make sure the two MS bits are 0
  }

  ret_val = ((float)int_temp.INT16) *
            TEMPERATURE_SENSOR_RESOLUTION; // Convert to float including the 2
                                           // bit shift
  return (ret_val);
}

// Configure the input multiplexer
boolean SFE_ADS122C04::setInputMultiplexer(uint8_t mux_config) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg0.bit.MUX = mux_config;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Configure the gain
boolean SFE_ADS122C04::setGain(uint8_t gain_config) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg0.bit.GAIN = gain_config;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Enable/disable the Programmable Gain Amplifier
boolean SFE_ADS122C04::enablePGA(uint8_t enable) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg0.bit.PGA_BYPASS = enable;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Set the data rate (sample speed)
boolean SFE_ADS122C04::setDataRate(uint8_t rate) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg1.bit.DR = rate;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the operating mode (normal / turbo)
boolean SFE_ADS122C04::setOperatingMode(uint8_t mode) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg1.bit.MODE = mode;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the conversion mode (single-shot / continuous)
boolean SFE_ADS122C04::setConversionMode(uint8_t mode) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg1.bit.CMBIT = mode;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the voltage reference
boolean SFE_ADS122C04::setVoltageReference(uint8_t ref) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg1.bit.VREF = ref;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Enable / disable the internal temperature sensor
boolean SFE_ADS122C04::enableInternalTempSensor(uint8_t enable) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg1.bit.TS = enable;

  return (ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Enable / disable the conversion data counter
boolean SFE_ADS122C04::setDataCounter(uint8_t enable) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg2.bit.DCNT = enable;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the data integrity check
boolean SFE_ADS122C04::setDataIntegrityCheck(uint8_t setting) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg2.bit.CRC = setting;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Enable / disable the 10uA burn-out current source
boolean SFE_ADS122C04::setBurnOutCurrent(uint8_t enable) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg2.bit.BCS = enable;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the internal programmable current sources
boolean SFE_ADS122C04::setIDACcurrent(uint8_t current) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg2.bit.IDAC = current;

  return (ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the IDAC1 routing
boolean SFE_ADS122C04::setIDAC1mux(uint8_t setting) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg3.bit.I1MUX = setting;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all));
}

// Configure the IDAC2 routing
boolean SFE_ADS122C04::setIDAC2mux(uint8_t setting) {
  if ((ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all)) ==
      false)
    return (false);
  ADS122C04_Reg.reg3.bit.I2MUX = setting;
  return (ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all));
}

// Read Config Reg 2 and check the DRDY bit
// Data is ready when DRDY is high
boolean SFE_ADS122C04::checkDataReady(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return (ADS122C04_Reg.reg2.bit.DRDY > 0);
}

// Get the input multiplexer configuration
uint8_t SFE_ADS122C04::getInputMultiplexer(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
  return (ADS122C04_Reg.reg0.bit.MUX);
}

// Get the gain setting
uint8_t SFE_ADS122C04::getGain(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
  return (ADS122C04_Reg.reg0.bit.GAIN);
}

// Get the Programmable Gain Amplifier status
uint8_t SFE_ADS122C04::getPGAstatus(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
  return (ADS122C04_Reg.reg0.bit.PGA_BYPASS);
}

// Get the data rate (sample speed)
uint8_t SFE_ADS122C04::getDataRate(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return (ADS122C04_Reg.reg1.bit.DR);
}

// Get the operating mode (normal / turbo)
uint8_t SFE_ADS122C04::getOperatingMode(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return (ADS122C04_Reg.reg1.bit.MODE);
}

// Get the conversion mode (single-shot / continuous)
uint8_t SFE_ADS122C04::getConversionMode(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return (ADS122C04_Reg.reg1.bit.CMBIT);
}

// Get the voltage reference configuration
uint8_t SFE_ADS122C04::getVoltageReference(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return (ADS122C04_Reg.reg1.bit.VREF);
}

// Get the internal temperature sensor status
uint8_t SFE_ADS122C04::getInternalTempSensorStatus(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);

  return (ADS122C04_Reg.reg1.bit.TS);
}

// Get the data counter status
uint8_t SFE_ADS122C04::getDataCounter(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return (ADS122C04_Reg.reg2.bit.DCNT);
}

// Get the data integrity check configuration
uint8_t SFE_ADS122C04::getDataIntegrityCheck(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return (ADS122C04_Reg.reg2.bit.CRC);
}

// Get the burn-out current status
uint8_t SFE_ADS122C04::getBurnOutCurrent(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return (ADS122C04_Reg.reg2.bit.BCS);
}

// Get the IDAC setting
uint8_t SFE_ADS122C04::getIDACcurrent(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);

  return (ADS122C04_Reg.reg2.bit.IDAC);
}

// Get the IDAC1 mux configuration
uint8_t SFE_ADS122C04::getIDAC1mux(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
  return (ADS122C04_Reg.reg3.bit.I1MUX);
}

// Get the IDAC2 mux configuration
uint8_t SFE_ADS122C04::getIDAC2mux(void) {
  ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
  return (ADS122C04_Reg.reg3.bit.I2MUX);
}

// Update ADS122C04_Reg and initialise the ADS122C04 using the supplied
// parameters
boolean SFE_ADS122C04::ADS122C04_init(ADS122C04_initParam *param) {
  ADS122C04_Reg.reg0.all =
      0; // Reset all four register values to the default value of 0x00
  ADS122C04_Reg.reg1.all = 0;
  ADS122C04_Reg.reg2.all = 0;
  ADS122C04_Reg.reg3.all = 0;

  ADS122C04_Reg.reg0.bit.MUX = param->inputMux;
  ADS122C04_Reg.reg0.bit.GAIN = param->gainLevel;
  ADS122C04_Reg.reg0.bit.PGA_BYPASS = param->pgaBypass;

  ADS122C04_Reg.reg1.bit.DR = param->dataRate;
  ADS122C04_Reg.reg1.bit.MODE = param->opMode;
  ADS122C04_Reg.reg1.bit.CMBIT = param->convMode;
  ADS122C04_Reg.reg1.bit.VREF = param->selectVref;
  ADS122C04_Reg.reg1.bit.TS = param->tempSensorEn;

  ADS122C04_Reg.reg2.bit.DCNT = param->dataCounterEn;
  ADS122C04_Reg.reg2.bit.CRC = param->dataCRCen;
  ADS122C04_Reg.reg2.bit.BCS = param->burnOutEn;
  ADS122C04_Reg.reg2.bit.IDAC = param->idacCurrent;

  ADS122C04_Reg.reg3.bit.I1MUX = param->routeIDAC1;
  ADS122C04_Reg.reg3.bit.I2MUX = param->routeIDAC2;

  boolean ret_val = true; // Flag to show if the four writeRegs were successful
  // (If any one writeReg returns false, ret_val will be false)
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all);
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all);
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all);
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all);

  return (ret_val);
}

boolean SFE_ADS122C04::reset(void) {
  return (ADS122C04_sendCommand(ADS122C04_RESET_CMD));
}

boolean SFE_ADS122C04::start(void) {
  return (ADS122C04_sendCommand(ADS122C04_START_CMD));
}

boolean SFE_ADS122C04::powerdown(void) {
  return (ADS122C04_sendCommand(ADS122C04_POWERDOWN_CMD));
}

boolean SFE_ADS122C04::ADS122C04_writeReg(uint8_t reg, uint8_t writeValue) {
  uint8_t command = 0;
  command = ADS122C04_WRITE_CMD(reg);
  return (ADS122C04_sendCommandWithValue(command, writeValue));
}

boolean SFE_ADS122C04::ADS122C04_readReg(uint8_t reg, uint8_t *readValue) {
  uint8_t command = 0;
  command = ADS122C04_READ_CMD(reg);

  _i2cPort->write(command);
  if (_i2cPort->available() >= 1) {
    *readValue = _i2cPort->read();
    return (true);
  }
  return (false);
}

boolean SFE_ADS122C04::ADS122C04_sendCommand(uint8_t command) {
  _i2cPort->write(command);
  return (true);
}

boolean SFE_ADS122C04::ADS122C04_sendCommandWithValue(uint8_t command,
                                                      uint8_t value) {
  _i2cPort->write(command);
  _i2cPort->write(value);
  return (true);
}

// Read the conversion result with count byte.
// The conversion result is 24-bit two's complement (signed)
// and is returned in the 24 lowest bits of the uint32_t conversionData.
// Hence it will always appear positive.
// Higher functions will need to take care of converting it to (e.g.) float or
// int32_t.
boolean
SFE_ADS122C04::ADS122C04_getConversionDataWithCount(uint32_t *conversionData,
                                                    uint8_t *count) {
  uint8_t RXByte[4] = {0};

  _i2cPort->write(ADS122C04_RDATA_CMD);

  // Note: the next line will need to be changed if data integrity is enabled.
  //       The code will need to request 6 bytes for CRC or 7 bytes for inverted
  //       data.

  if (_i2cPort->available() >= 4) {
    RXByte[0] = _i2cPort->read(); // Count
    RXByte[1] = _i2cPort->read(); // MSB
    RXByte[2] = _i2cPort->read();
    RXByte[3] = _i2cPort->read();  // LSB
    if (_i2cPort->available() > 0) // Note: this _should_ be redundant
    {

      while (_i2cPort->available() > 0) {
        _i2cPort->read(); // Read and ignore excess bytes (presumably inverted
                          // data or CRC)
      }
    }
  } else {

    return (false);
  }

  *count = RXByte[0];
  *conversionData = ((uint32_t)RXByte[3]) | ((uint32_t)RXByte[2] << 8) |
                    ((uint32_t)RXByte[1] << 16);
  return (true);
}

// Read the conversion result.
// The conversion result is 24-bit two's complement (signed)
// and is returned in the 24 lowest bits of the uint32_t conversionData.
// Hence it will always appear positive.
// Higher functions will need to take care of converting it to (e.g.) float or
// int32_t.
boolean SFE_ADS122C04::ADS122C04_getConversionData(uint32_t *conversionData) {
  uint8_t RXByte[3] = {0};

  _i2cPort->write(ADS122C04_RDATA_CMD);

  // Note: the next line will need to be changed if data integrity is enabled.
  //       The code will need to request 5 bytes for CRC or 6 bytes for inverted
  //       data.

  if (_i2cPort->available() >= 3) {
    RXByte[0] = _i2cPort->read(); // MSB
    RXByte[1] = _i2cPort->read();
    RXByte[2] = _i2cPort->read();  // LSB
    if (_i2cPort->available() > 0) // Note: this _should_ be redundant
    {

      while (_i2cPort->available() > 0) {
        _i2cPort->read(); // Read and ignore excess bytes (presumably inverted
                          // data or CRC)
      }
    }
  } else {

    return (false);
  }

  *conversionData = ((uint32_t)RXByte[2]) | ((uint32_t)RXByte[1] << 8) |
                    ((uint32_t)RXByte[0] << 16);
  return (true);
}
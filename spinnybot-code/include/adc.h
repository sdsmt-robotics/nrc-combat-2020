#ifndef ADC_H
#define ADC_H

#include "Arduino.h"

// Single Conversion Timeout (millis)
// The maximum time we will wait for DRDY to go valid for a single conversion
#define ADS122U04_CONVERSION_TIMEOUT 75

// Define 2/3/4-Wire, Temperature and Raw modes
#define ADS122U04_4WIRE_MODE 0x0
#define ADS122U04_3WIRE_MODE 0x1
#define ADS122U04_2WIRE_MODE 0x2
#define ADS122U04_TEMPERATURE_MODE 0x3
#define ADS122U04_RAW_MODE 0x4
#define ADS122U04_4WIRE_HI_TEMP 0x5
#define ADS122U04_3WIRE_HI_TEMP 0x6
#define ADS122U04_2WIRE_HI_TEMP 0x7

// ADS122U04 Table 16 in Datasheet
#define ADS122U04_RESET_CMD 0x06     // 0000 011x      Reset
#define ADS122U04_START_CMD 0x08     // 0000 100x      Start/Sync
#define ADS122U04_POWERDOWN_CMD 0x02 // 0000 001x      PowerDown
#define ADS122U04_RDATA_CMD 0x10     // 0001 xxxx      RDATA
#define ADS122U04_RREG_CMD                                                     \
  0x20 // 0010 rrxx      Read REG rr= register address 00 to 11
#define ADS122U04_WREG_CMD                                                     \
  0x40 // 0100 rrxx      Write REG rr= register address 00 to 11

#define ADS122U04_WRITE_CMD(reg)                                               \
  (ADS122U04_WREG_CMD | (reg << 2)) // Shift is 2-bit in ADS122U04
#define ADS122U04_READ_CMD(reg)                                                \
  (ADS122U04_RREG_CMD | (reg << 2)) // Shift is 2-bit in ADS122U04

// ADS122U04 Table 16 in Datasheet
#define ADS122U04_CONFIG_0_REG 0 // Configuration Register 0
#define ADS122U04_CONFIG_1_REG 1 // Configuration Register 1
#define ADS122U04_CONFIG_2_REG 2 // Configuration Register 2
#define ADS122U04_CONFIG_3_REG 3 // Configuration Register 3

// Unshifted register definitions
// The bit field register definitions will do the bit shifting

// Configuration Register 0
// ADS122U04 Table 19 in Datasheet

// Input Multiplexer Configuration
#define ADS122U04_MUX_AIN0_AIN1 0x0
#define ADS122U04_MUX_AIN0_AIN2 0x1
#define ADS122U04_MUX_AIN0_AIN3 0x2
#define ADS122U04_MUX_AIN1_AIN0 0x3
#define ADS122U04_MUX_AIN1_AIN2 0x4
#define ADS122U04_MUX_AIN1_AIN3 0x5
#define ADS122U04_MUX_AIN2_AIN3 0x6
#define ADS122U04_MUX_AIN3_AIN2 0x7
#define ADS122U04_MUX_AIN0_AVSS 0x8
#define ADS122U04_MUX_AIN1_AVSS 0x9
#define ADS122U04_MUX_AIN2_AVSS 0xa
#define ADS122U04_MUX_AIN3_AVSS 0xb
#define ADS122U04_MUX_REFPmREFN 0xc
#define ADS122U04_MUX_AVDDmAVSS 0xd
#define ADS122U04_MUX_SHORTED 0xe

// Gain Configuration
#define ADS122U04_GAIN_1 0x0
#define ADS122U04_GAIN_2 0x1
#define ADS122U04_GAIN_4 0x2
#define ADS122U04_GAIN_8 0x3
#define ADS122U04_GAIN_16 0x4
#define ADS122U04_GAIN_32 0x5
#define ADS122U04_GAIN_64 0x6
#define ADS122U04_GAIN_128 0x7

// PGA Bypass (PGA is disabled when the PGA_BYPASS bit is set)
#define ADS122U04_PGA_DISABLED 0x1
#define ADS122U04_PGA_ENABLED 0x0

// Configuration Register 1
// ADS122U04 Table 19 in Datasheet

// Data Rate
// Turbo mode = Normal mode * 2 (Samples per Second)
// Normal mode
#define ADS122U04_DATA_RATE_20SPS 0x0
#define ADS122U04_DATA_RATE_45SPS 0x1
#define ADS122U04_DATA_RATE_90SPS 0x2
#define ADS122U04_DATA_RATE_175SPS 0x3
#define ADS122U04_DATA_RATE_330SPS 0x4
#define ADS122U04_DATA_RATE_600SPS 0x5
#define ADS122U04_DATA_RATE_1000SPS 0x6

// Operating Mode
#define ADS122U04_OP_MODE_NORMAL 0x0
#define ADS122U04_OP_MODE_TURBO 0x1

// Conversion Mode
#define ADS122U04_CONVERSION_MODE_SINGLE_SHOT 0x0
#define ADS122U04_CONVERSION_MODE_CONTINUOUS 0x1

// Voltage Reference Selection
#define ADS122U04_VREF_INTERNAL 0x0     // 2.048V internal
#define ADS122U04_VREF_EXT_REF_PINS 0x1 // REFp and REFn external
#define ADS122U04_VREF_AVDD 0x2         // Analog Supply AVDD and AVSS

// Temperature Sensor Mode
#define ADS122U04_TEMP_SENSOR_OFF 0x0
#define ADS122U04_TEMP_SENSOR_ON 0x1

// Configuration Register 2
// ADS122U04 Table 22 in Datasheet

// Conversion Result Ready Flag (READ ONLY)

// Data Counter Enable
#define ADS122U04_DCNT_DISABLE 0x0
#define ADS122U04_DCNT_ENABLE 0x1

// Data Integrity Check Enable
#define ADS122U04_CRC_DISABLED 0x0
#define ADS122U04_CRC_INVERTED 0x1
#define ADS122U04_CRC_CRC16_ENABLED 0x2

// Burn-Out Current Source
#define ADS122U04_BURN_OUT_CURRENT_OFF 0x0
#define ADS122U04_BURN_OUT_CURRENT_ON 0x1

// IDAC Current Setting
#define ADS122U04_IDAC_CURRENT_OFF 0x0
#define ADS122U04_IDAC_CURRENT_10_UA 0x1
#define ADS122U04_IDAC_CURRENT_50_UA 0x2
#define ADS122U04_IDAC_CURRENT_100_UA 0x3
#define ADS122U04_IDAC_CURRENT_250_UA 0x4
#define ADS122U04_IDAC_CURRENT_500_UA 0x5
#define ADS122U04_IDAC_CURRENT_1000_UA 0x6
#define ADS122U04_IDAC_CURRENT_1500_UA 0x7

// Configuration Register 3
// ADS122U04 Table 23 in Datasheet

// IDAC1 Routing Configuration
#define ADS122U04_IDAC1_DISABLED 0x0
#define ADS122U04_IDAC1_AIN0 0x1
#define ADS122U04_IDAC1_AIN1 0x2
#define ADS122U04_IDAC1_AIN2 0x3
#define ADS122U04_IDAC1_AIN3 0x4
#define ADS122U04_IDAC1_REFP 0x5
#define ADS122U04_IDAC1_REFN 0x6

// IDAC2 Routing Configuration
#define ADS122U04_IDAC2_DISABLED 0x0
#define ADS122U04_IDAC2_AIN0 0x1
#define ADS122U04_IDAC2_AIN1 0x2
#define ADS122U04_IDAC2_AIN2 0x3
#define ADS122U04_IDAC2_AIN3 0x4
#define ADS122U04_IDAC2_REFP 0x5
#define ADS122U04_IDAC2_REFN 0x6

// Bit field type register configuration
// Configuration Map register ADS122U04
//--------------Address 0x00---------------------------------
struct CONFIG_REG_0 {
  uint8_t PGA_BYPASS : 1; // 0
  uint8_t GAIN : 3;       // 1-3
  uint8_t MUX : 4;        // 4-7
};
union CONFIG_REG_0_U {
  uint8_t all;
  struct CONFIG_REG_0 bit;
};

//--------------Address 0x01---------------------------------
struct CONFIG_REG_1 {
  uint8_t TS : 1;    // 0
  uint8_t VREF : 2;  // 1-2
  uint8_t CMBIT : 1; // 3
  uint8_t MODE : 1;  // 4
  uint8_t DR : 3;    // 5-7
};
union CONFIG_REG_1_U {
  uint8_t all;
  struct CONFIG_REG_1 bit;
};

//--------------Address 0x02---------------------------------
struct CONFIG_REG_2 {
  uint8_t IDAC : 3; // 0-2
  uint8_t BCS : 1;  // 3
  uint8_t CRC : 2;  // 4-5
  uint8_t DCNT : 1; // 6
  uint8_t DRDY : 1; // 7
};
union CONFIG_REG_2_U {
  uint8_t all;
  struct CONFIG_REG_2 bit;
};

//--------------Address 0x03---------------------------------
struct CONFIG_REG_3 {
  uint8_t RESERVED : 2; // 0-1
  uint8_t I2MUX : 3;    // 2-4
  uint8_t I1MUX : 3;    // 5-7
};
union CONFIG_REG_3_U {
  uint8_t all;
  struct CONFIG_REG_3 bit;
};

// All four registers
typedef struct ADS122U04Reg {
  union CONFIG_REG_0_U reg0;
  union CONFIG_REG_1_U reg1;
  union CONFIG_REG_2_U reg2;
  union CONFIG_REG_3_U reg3;
} ADS122U04Reg_t;

// Union for the 14-bit internal Temperature
// To simplify converting from uint16_t to int16_t
// without using a cast
union internal_temperature_union {
  int16_t INT16;
  uint16_t UINT16;
};

// Union for the 24-bit raw voltage
// To simplify converting from uint32_t to int32_t
// without using a cast
union raw_voltage_union {
  int32_t INT32;
  uint32_t UINT32;
};

// struct to hold the initialisation parameters
typedef struct {
  uint8_t inputMux;
  uint8_t gainLevel;
  uint8_t pgaBypass;
  uint8_t dataRate;
  uint8_t opMode;
  uint8_t convMode;
  uint8_t selectVref;
  uint8_t tempSensorEn;
  uint8_t dataCounterEn;
  uint8_t dataCRCen;
  uint8_t burnOutEn;
  uint8_t idacCurrent;
  uint8_t routeIDAC1;
  uint8_t routeIDAC2;
} ADS122U04_initParam;

class ADS122U04 {
public:
  ADS122U04(HardwareSerial *serial);

  // By default use the default I2C address, and use Wire port
  bool begin(int baud, int rx, int tx); // Returns true if module is detected

  // float readPT100Centigrade(void); // Read the PT100 temperature in
  // Centigrade float readPT100Fahrenheit(void); // Read the PT100 temperature
  // in Fahrenheit

  // Read the raw signed 24-bit ADC value as int32_t
  // This uses the internal 2.048V reference with the gain set to 1
  // The LSB is 2.048 / 2^23 = 0.24414 uV (0.24414 microvolts)
  int32_t readRawVoltage(uint8_t rate = ADS122U04_DATA_RATE_20SPS);

  // Read the raw signed 24-bit ADC value as uint32_t
  // The ADC data is returned in the least-significant 24-bits
  uint32_t readADC(void);

  // Read the internal temperature (C)
  float readInternalTemperature(uint8_t rate = ADS122U04_DATA_RATE_20SPS);

  boolean reset(void);     // Reset the ADS122U04
  boolean start(void);     // Start a conversion
  boolean powerdown(void); // Put the chip into low power mode

  // Default to using 'safe' settings (disable the IDAC current sources)
  boolean configureADCmode(
      uint8_t wire_mode = ADS122U04_RAW_MODE,
      uint8_t rate =
          ADS122U04_DATA_RATE_20SPS); // Configure the chip for the chosen mode

  // Default to using 'safe' settings (disable the IDAC current sources)
  boolean setInputMultiplexer(
      uint8_t mux_config =
          ADS122U04_MUX_AIN1_AIN0); // Configure the input multiplexer
  boolean setGain(uint8_t gain_config = ADS122U04_GAIN_1); // Configure the gain
  boolean enablePGA(
      uint8_t enable = ADS122U04_PGA_DISABLED); // Enable/disable the
                                                // Programmable Gain Amplifier
  boolean setDataRate(
      uint8_t rate =
          ADS122U04_DATA_RATE_20SPS); // Set the data rate (sample speed)
  boolean setOperatingMode(
      uint8_t mode = ADS122U04_OP_MODE_NORMAL); // Configure the operating mode
                                                // (normal / turbo)
  boolean setConversionMode(
      uint8_t mode =
          ADS122U04_CONVERSION_MODE_SINGLE_SHOT); // Configure the conversion
                                                  // mode (single-shot /
                                                  // continuous)
  boolean setVoltageReference(
      uint8_t ref = ADS122U04_VREF_INTERNAL); // Configure the voltage reference
  boolean enableInternalTempSensor(
      uint8_t enable =
          ADS122U04_TEMP_SENSOR_OFF); // Enable / disable the internal
                                      // temperature sensor
  boolean setDataCounter(
      uint8_t enable = ADS122U04_DCNT_DISABLE); // Enable / disable the
                                                // conversion data counter
  boolean setDataIntegrityCheck(
      uint8_t setting =
          ADS122U04_CRC_DISABLED); // Configure the data integrity check
  boolean setBurnOutCurrent(
      uint8_t enable =
          ADS122U04_BURN_OUT_CURRENT_OFF); // Enable / disable the 10uA burn-out
                                           // current source
  boolean setIDACcurrent(
      uint8_t current =
          ADS122U04_IDAC_CURRENT_OFF); // Configure the internal programmable
                                       // current sources
  boolean
  setIDAC1mux(uint8_t setting =
                  ADS122U04_IDAC1_DISABLED); // Configure the IDAC1 routing
  boolean
  setIDAC2mux(uint8_t setting =
                  ADS122U04_IDAC2_DISABLED); // Configure the IDAC2 routing

  boolean
  checkDataReady(void); // Check the status of the DRDY bit in Config Register 2

  uint8_t getInputMultiplexer(void); // Get the input multiplexer configuration
  uint8_t getGain(void);             // Get the gain setting
  uint8_t getPGAstatus(void);     // Get the Programmable Gain Amplifier status
  uint8_t getDataRate(void);      // Get the data rate (sample speed)
  uint8_t getOperatingMode(void); // Get the operating mode (normal / turbo)
  uint8_t
  getConversionMode(void); // Get the conversion mode (single-shot / continuous)
  uint8_t getVoltageReference(void); // Get the voltage reference configuration
  uint8_t getInternalTempSensorStatus(
      void);                    // Get the internal temperature sensor status
  uint8_t getDataCounter(void); // Get the data counter status
  uint8_t
  getDataIntegrityCheck(void);     // Get the data integrity check configuration
  uint8_t getBurnOutCurrent(void); // Get the burn-out current status
  uint8_t getIDACcurrent(void);    // Get the IDAC setting
  uint8_t getIDAC1mux(void);       // Get the IDAC1 mux configuration
  uint8_t getIDAC2mux(void);       // Get the IDAC2 mux configuration

  // Print the ADS122U04 configuration (but only if enableDebugging has been
  // called)
  // void printADS122U04config(void);

  // Requested in #5
  uint8_t getWireMode(void) { return (_wireMode); }

private:
  // Variables
  HardwareSerial
      *_serial; // The generic connection to user's chosen I2C hardware

  // Keep a copy of the wire mode so we can restore it after reading the
  // internal temperature
  uint8_t _wireMode = ADS122U04_4WIRE_MODE;

  // Resistance of the reference resistor
  const float PT100_REFERENCE_RESISTOR = 1620.0;

  // Amplifier gain setting
  // ** MAKE SURE THE CONFIG REGISTER 0 GAIN IS THE SAME AS THIS **
  const float PT100_AMPLIFIER_GAIN = 8.0;
  const float PT100_AMP_GAIN_HI_TEMP = 4.0;

  // Internal temperature sensor resolution
  // One 14-bit LSB equals 0.03125°C
  const float TEMPERATURE_SENSOR_RESOLUTION = 0.03125;

  ADS122U04Reg_t ADS122U04_Reg; // Global to hold copies of all four
                                // configuration registers

  boolean
  ADS122U04_init(ADS122U04_initParam
                     *param); // initialise the ADS122U04 with these parameters

  boolean ADS122U04_writeReg(
      uint8_t reg,
      uint8_t writeValue); // write a value to the selected register
  boolean
  ADS122U04_readReg(uint8_t reg,
                    uint8_t *readValue); // read a value from the selected
                                         // register (returned in readValue)

  boolean ADS122U04_getConversionData(
      uint32_t *conversionData); // read the raw 24-bit conversion result
  boolean ADS122U04_getConversionDataWithCount(
      uint32_t *conversionData,
      uint8_t *count); // read the raw conversion result and count (if enabled)

  boolean ADS122U04_sendCommand(
      uint8_t command); // write to the selected command register
  boolean ADS122U04_sendCommandWithValue(
      uint8_t command,
      uint8_t value); // write a value to the selected command register
};
#endif
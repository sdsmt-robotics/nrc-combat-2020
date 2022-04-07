/**
 * File to define the pins used by the various devices.
 */

#ifndef __BB_PINS__
#define __BB_PINS__

// ---- PINS ----

#define STATUS_LED_PIN 5
#define ESP_STATUS_LED_PIN 2 // internal led on esp32 (its the blue one)

#define LED_STRIP_1 26
#define LED_STRIP_2 27
#define LED_STRIP_3 14

#define ENC_1 36
#define ENC_2 39
#define ENC_3 34

#define ADC_SELECT 23
#define ADC_RX 19
#define ADC_TX 18

#define MOTOR_1 15
#define MOTOR_1_RMT_CHANNEL 3
#define MOTOR_2 33
#define MOTOR_2_RMT_CHANNEL 4
#define MOTOR_3 25
#define MOTOR_3_RMT_CHANNEL 5

#define CONTROLLER_SERIAL Serial2

#define ADC_SELECT 23
#define ADC_RX 19
#define ADC_TX 18

// ---- MISC ----

#define ADAFRUIT_RMT_CHANNEL_MAX 3 // LED strips can use 3 RMT channels 0 - 2

#endif
#include "motor.h"

/*
 * DSHOT600 protocol
 * https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
 *
 * 600 = speed in kbits / second
 *
 * 600khz means 1.66 us per bit,
 *
 * Each bit is transmitted as a high/low pulse
 * 0 = 555ns high, 1111ns low
 * 1 = 1111ns high, 555ns low
 *
 * So the clock rate we need is 1.8mhz
 *
 * the internal clock for the rmt peripheral is 80mhz
 * so clk_div could be 80 /1.8 = 44
 *
 * OR set clk_div to 1, and use 44 as a value for the pulses
 * short_time = 44 ticks, ( rounding down)
 * long_time = 89 ticks (rounding up)
 *
 * DSHOT150
 * 150 = speed in kbits / sec
 * 6.6us per bit
 * high time = 4.4us, 355 80mhz ticks
 * low time = 2.2us, 178 80mhz ticks
 *
 */

// 80mhz / CLOCK_DIVIDER / (PULSE_SHORT_TIME + PULSE_LONG_TIME)
// Must equal 1/600 sec

#define CLOCK_DIVIDER 1

// dshot600
#define PULSE_SHORT_TIME 44
#define PULSE_LONG_TIME 89
// dshot 150
// #define PULSE_SHORT_TIME 178
// #define PULSE_LONG_TIME 355

/*
 * Taken from https://github.com/gueei/DShot-Arduino/blob/master/src/DShot.cpp
 */
// top_12_bits should contain the 11 bits of throttle,
// and 1 bit of telemetry request (least significant)
// Throttle values 1..47 are reserved for commands,
// 0=stop, 1024 = half, 2047 = max
// This function shifts top_12_bits left 4 bits and fills in the crc
// in the least significant 4 bits.
uint16_t Motor::calc_crc(uint16_t top_12_bits) {
  uint8_t csum = 0;
  uint16_t csum_data = top_12_bits;
  for (uint8_t i = 0; i < 3; i++) {
    csum ^= csum_data;
    csum_data >>= 4;
  }
  csum &= 0xf;
  return (top_12_bits << 4) | csum;
}

void Motor::test_crc() {
  printf("Testing calc_crc\n");
  // Test vector throttle = 1046
  // telemetry bit =0
  // crc = 0110 (0x6)
  uint16_t throttle = 1046;
  uint16_t crctest = calc_crc(throttle << 1);
  printf("throttle = %04hx value with crc = %04hx\n", throttle, crctest);
  assert(crctest == 0x82c6);
}

void Motor::init_rmt() {
  /* Initialise RMT (remote control) peripheral.
   */
  rmt_config_t config;
  memset(&config, 0, sizeof(config)); // initialise unused fields to 0
  config.rmt_mode = RMT_MODE_TX;
  config.channel = _rmt_channel;
  config.gpio_num = _gpio_pin;
  config.mem_block_num = 1;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = rmt_idle_level_t::RMT_IDLE_LEVEL_LOW;
  config.clk_div = CLOCK_DIVIDER;
  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}

void Motor::send_pulses(uint16_t pulses_int) {

  uint16_t cmdbits = pulses_int;
  // Build 16-bit (32 pulses) of data
  rmt_item32_t pulses[16];
  // Iterate through bits, most-significant first.
  for (uint8_t i = 0; i < 16; i++) {
    pulses[i].level0 = 1;
    pulses[i].level1 = 0;
    if (cmdbits & 0x8000) {
      // bit set
      pulses[i].duration0 = PULSE_LONG_TIME;
      pulses[i].duration1 = PULSE_SHORT_TIME;
    } else {
      // bit clear
      pulses[i].duration0 = PULSE_SHORT_TIME;
      pulses[i].duration1 = PULSE_LONG_TIME;
    }
    cmdbits = cmdbits << 1; // Shift bits left
  }
  // Write and wait.
  esp_err_t err = rmt_write_items(_rmt_channel, pulses, 16, true);
  ESP_ERROR_CHECK(err);
}

void Motor::transmit_command(uint16_t top_12_bits) {
  uint16_t command = calc_crc(top_12_bits);
  send_pulses(command);
}

void Motor::init() { init_rmt(); }

void Motor::set_speed_signed(int speed_signed) {
  // dshot commands:
  // 0= off
  // 1..47 reserved (config etc)
  // 48..1047 = forward speeds
  // 1049..2047 = reverse speeds
  // 1048=?
  uint16_t dshot = 0; // motor off
  if (speed_signed > 0) {
    dshot = 47 + speed_signed;
    if (dshot > 1047)
      dshot = 1047;
  }
  if (speed_signed < 0) {
    dshot = 1048 - speed_signed;
    if (dshot > 2047)
      dshot = 2047;
  }
  uint16_t top_12_bits = (dshot << 1);
  transmit_command(top_12_bits);
}

void Motor::send_dshot_command(int cmd) {
  uint16_t top_12_bits = (cmd << 1);
  top_12_bits |= 1;
  transmit_command(top_12_bits);
}

Motor::Motor(int pin, int channel) {
  _rmt_channel = static_cast<rmt_channel_t>(channel);
  _gpio_pin = static_cast<gpio_num_t>(pin);
};

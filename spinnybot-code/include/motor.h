#include <stdbool.h>
#include <stdint.h>

#include "driver/rmt.h"

#ifndef MOTORS_H
#define MOTORS_H

#define DSHOT_CMD_3D_MODE_ON 10

class Motor {

  rmt_channel_t _rmt_channel;
  gpio_num_t _gpio_pin;

public:
  Motor(int pin, int channel);

  void init();

  // Motor is either 0 or 1
  // Speed is in range -1000..1000
  // This function sends the command immediately then returns.
  void set_speed_signed(int speed_signed);

  void send_dshot_command(int cmd);

private:
  uint16_t calc_crc(uint16_t top_12_bits);
  void test_crc();
  void init_rmt();
  void send_pulses(uint16_t pulses_int);
  void transmit_command(uint16_t top_12_bits);
};

#endif

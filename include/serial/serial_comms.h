
#include <cstdint>
#include <cstring>
#include "serial_protocol.h"

void setup_serial_uart();
// int read_from_serial(uint8_t * data, uint32_t size);
int write_to_serial(uint8_t * data, uint32_t size);

int write_b_to_serial(uint8_t b);
int write_f_to_serial(float f);
int write_u_to_serial(uint32_t u);
int write_i_to_serial(int32_t i);

inline float bits_to_f32(uint8_t *bits) {
  float f;
  memcpy(&f, bits, 4);
  return f;
}

// returns length of message.
// Written message is null terminated and stripped of
// MSG_START and MSG_END.
int receive_incoming_message(uint8_t * message, uint32_t max_size);

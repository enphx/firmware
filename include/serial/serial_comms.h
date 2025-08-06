#include "include/constants.h"

#ifdef SERIAL_OUTPUT

#include <cstdint>
#include <cstring>
#include "serial_protocol.h"

void setup_serial_uart();
// int read_from_serial(uint8_t * data, uint32_t size);
int write_to_serial(const void * data, uint32_t size);

int write_b_to_serial(uint8_t b);
int write_f_to_serial(double f);
int write_u_to_serial(uint32_t u);
int write_i_to_serial(int32_t i);

inline double bits_to_f32(uint8_t *bits) {
  double f;
  memcpy(&f, bits, 4);
  return f;
}



inline void copy_4(uint8_t *target, void *source, uint32_t *index) {
  *index = *index + 4;
  memcpy(target, source, 4);
}

inline void copy_1(uint8_t *target, uint8_t source, uint32_t *index) {
  *index = *index + 1;
  target[0] = source;
}

inline void copy_f(uint8_t *target, double f, uint32_t *index) {
  copy_1(target, FLOAT_AHEAD, index);
  copy_4(&target[1], &f, index);
}

// returns length of message.
// Written message is null terminated and stripped of
// MSG_START and MSG_END.
int receive_incoming_message(uint8_t * message, uint32_t max_size);

#endif

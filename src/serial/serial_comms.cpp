#include "include/serial/serial_comms.h"
#include "driver/uart.h"
#include <Arduino.h>

const uart_port_t uart_num = UART_NUM_1;
#define PIN_TX 1
#define PIN_RX 3

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

uint8_t data[256]; // message buffer.
uint32_t i = 0;


int read_from_serial(uint8_t * data, uint32_t size) {
  return uart_read_bytes(uart_num, data, size, 0);
}

int write_to_serial(uint8_t * data, uint32_t size) {
  return uart_write_bytes(uart_num, data, size);
}


int write_b_to_serial(uint8_t b) {
  return uart_write_bytes(uart_num, &b, 1);
}

int write_f_to_serial(float f) {
  uint8_t tag = FLOAT_AHEAD;
  uart_write_bytes(uart_num, &tag, 1);
  return uart_write_bytes(uart_num,(uint8_t *)&f, sizeof(float));
}

int write_u_to_serial(uint32_t u) {
  uint8_t tag = UINT_AHEAD;
  uart_write_bytes(uart_num, &tag, 1);
  return uart_write_bytes(uart_num, (uint8_t *)&u, 4);
}

int write_i_to_serial(int32_t i) {
  uint8_t tag = INT_AHEAD;
  uart_write_bytes(uart_num, &tag, 1);
    return uart_write_bytes(uart_num, (uint8_t *) &i, 4);
}

void setup_serial_uart() {
  ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024, 1024, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


int receive_incoming_message(uint8_t * message, uint32_t max_size) {
  // write_to_serial(data, 256);

  // Overwrite the message if it gets too long.
  if (i >= 250 || data[0] != MSG_START) {
    i = 0;
  }

  int last = read_from_serial(&(data[i]), 256 - i);
  if (i + last - 2 > max_size) {
    i = 0;
    return 0;
  }
  if (data[i + last - 1] == MSG_END) {
    memcpy(message, &(data[1]), i + last - 2);
    i = 0;
    return (i + last - 2);
  } else {
    i += last;
    return 0;
  }
}

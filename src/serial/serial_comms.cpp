#include "include/constants.h"

#ifdef SERIAL_OUTPUT

#include "include/serial/serial_comms.h"
#include "driver/uart.h"
#include <stdio.h>
#include <Arduino.h>
#include <cstdio>
#include <cstring>

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

int write_to_serial(const void * data, uint32_t size) {
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

int uart_vprintf(const char *fmt, va_list args) {
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len > 0) {
        uart_write_bytes(UART_NUM_0, buf, len);
    }
    return len;
}

void setup_serial_uart() {
  ESP_ERROR_CHECK(uart_driver_install(uart_num, 2048, 2048, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  esp_log_set_vprintf(uart_vprintf);
}

#define BUF_SIZE 1024
static uint8_t serial_buffer[BUF_SIZE] = {0};
static uint32_t buf_index = 0;

int receive_incoming_message(uint8_t * message, uint32_t max_size) {
  if (buf_index >= BUF_SIZE) {
    buf_index = 0;
  }

  int new_vals = read_from_serial(&serial_buffer[buf_index], BUF_SIZE - buf_index);

  buf_index += new_vals;

  if (new_vals < 0) {
    return 0;
  }


  int start_i;
  int end_i;

  for (start_i = 0; serial_buffer[start_i] != MSG_START && start_i < buf_index; start_i++) ;

  if (start_i >= buf_index) {
    buf_index = 0;
    return 0;
  }


  for (end_i = start_i; serial_buffer[end_i] != MSG_END && end_i < buf_index; end_i++) ;

  if (end_i >= buf_index) {
    return 0;
  }


  int msg_size = end_i - start_i - 1;
  msg_size = msg_size < max_size ? msg_size : max_size;

  memcpy(message, &serial_buffer[start_i + 1], msg_size);

  if (end_i + 1 < buf_index) {
    memmove(serial_buffer, &serial_buffer[end_i + 1], buf_index - end_i - 1);
    buf_index = buf_index - end_i - 1;
  } else {
    buf_index = 0;
  }

  return msg_size;
}

#endif

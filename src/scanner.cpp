
#include "include/scanner.h"
#include "include/constants.h"
#include "include/serial/serial_comms.h"

static const char* TAG = "SCANNER";

ScannerPoint Scanner::push(int16_t distance, RobotPosition robot_position) {
  if (first_value) {
    for (int i = 0; i < KERN_SIZE; i++) {
      conv_filter.eval(distance);
    }
    first_value = false;
  }

  ScannerPoint output = {
    distance,
    conv_filter.eval(distance),
    robot_position,
  };

  buffer[buf_index] = output;
  buf_index = (buf_index + 1) >= BUF_SIZE ? BUF_SIZE - 1 : buf_index + 1;
 
  return output;
}

void Scanner::print_log() {
  #ifdef SERIAL_OUTPUT

  uint8_t message_to_send[256];

  for (int j = 0; j < buf_index; j++) {
    uint32_t i = 0;

    copy_1(&message_to_send[i], MSG_START, &i);
    copy_1(&message_to_send[i], LIDAR, &i);
    copy_1(&message_to_send[i], ALL, &i);
    
    copy_f(&message_to_send[i], buffer[j].distance, &i);
    copy_f(&message_to_send[i], buffer[j].convolved, &i);

    copy_1(&message_to_send[i], MSG_END, &i);

    write_to_serial(message_to_send, i);
    vTaskDelay(1);
  }

  #else

  ESP_LOGI(TAG, "angle, distance, convolved");
  for (int i = 0; i < buf_index; i++) {
    ESP_LOGI(TAG, "%f, %i, %i", buffer[i].distance, buffer[i].convolved);
  }
 
  #endif
}

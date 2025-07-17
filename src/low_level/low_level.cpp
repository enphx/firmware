#include "core0.h"
#include "esp32-hal-log.h"
#include "include/low_level.h"
#include <Arduino.h>

static const char* TAG = "LOW LEVEL";

void low_level_init() {
  ESP_LOGI("HAL", "init...");
  core0_init();
}

void print_adc_vals() {
  ESP_LOGI(TAG, "0: %hu, 1: %hu, 2: %hu, 3: %hu, 4: %hu, 5: %hu, 6: %hu, 7: %hu,",
           get_convolved_value(0),
           get_convolved_value(1),
           get_convolved_value(2),
           get_convolved_value(3),
           get_convolved_value(4),
           get_convolved_value(5),
           get_convolved_value(6),
           get_convolved_value(7)
         );
}

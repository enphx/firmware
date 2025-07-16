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
  for (uint16_t i = 0; i < 8; i++) {
    ESP_LOGI(TAG, "ch: %hu, val: %hu", i, get_convolved_value(i));
  }
  ESP_LOGI(TAG, "Done printing adc values.");
}

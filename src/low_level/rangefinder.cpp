#include "include/low_level/rangefinder.h"
#include "include/low_level/io.h"


static const char* TAG = "RANGEFINDER";

RangeFinder::RangeFinder(uint8_t i2c_addr) : addr(i2c_addr)  {}

void RangeFinder::init() {
  if (!vl53.begin(I2C_RANGEFINDER_ADDR)) {
    while (1) {
      ESP_LOGE(TAG, "Couldn't start ranging...");
      vTaskDelay(1000);
    }
  } else {
    ESP_LOGI(TAG, "Rangefinder found successfully at address %u", addr);
  }

  if (!vl53.startRanging()) {
    while (1) {
      ESP_LOGE(TAG, "Couldn't start ranging...");
      vTaskDelay(1000);
    }
  }

  vl53.setTimingBudget(50);

  ESP_LOGI(TAG, "Started ranging successfully!!");
}

int16_t RangeFinder::getDistance() {
  return vl53.distance();
}

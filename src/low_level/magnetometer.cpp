#include "include/low_level/magnetometer.h"
#include "Adafruit_LIS3MDL.h"
#include <Arduino.h>

static const char * TAG = "MAGNETOMETER";

Magnetometer::Magnetometer(uint8_t i2c_addr) : addr(i2c_addr) {
  
}

void Magnetometer::init() {
  if (!lis3mdl.begin_I2C(addr, &Wire)) {
    ESP_LOGE(TAG, "Failed to start magnetometer I2C at addr %u...", addr);
    while (1) {
      vTaskDelay(1000);
      ESP_LOGE(TAG, "Failed to start magnetometer I2C at addr %u...", addr);
    }
  }
  ESP_LOGI(TAG, "Successfully found magnetometer at I2C addr %u", addr);

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);

  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  lis3mdl.setIntThreshold(500);

  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true,               // polarity
                          false,              // don't latch
                          true);              // enabled!

  
}

magVector Magnetometer::getMagneticVector() {
  lis3mdl.read();
  return magVector {lis3mdl.x, lis3mdl.y, lis3mdl.z};
}

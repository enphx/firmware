#ifndef RANGE_FINDER_H
#define RANGE_FINDER_H

#include <Arduino.h>

#include "Adafruit_VL53L1X.h"

class RangeFinder{
public:
  RangeFinder(uint8_t i2c_addr);
  void init();

  float getDistance();

private:
  const uint8_t addr;
  Adafruit_VL53L1X vl53;
};

#endif

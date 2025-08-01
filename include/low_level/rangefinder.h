#ifndef RANGE_FINDER_H
#define RANGE_FINDER_H

#include <Arduino.h>

#include "Adafruit_VL53L1X.h"

#define MAX_DISTANCE_VALUE 4500

class RangeFinder{
public:
  RangeFinder(uint8_t i2c_addr);
  void init();

  int16_t getDistance();

  inline bool getDataReady() {
    return vl53.dataReady();
  }

private:
  const uint8_t addr;
  Adafruit_VL53L1X vl53;
};

#endif

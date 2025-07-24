#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include "Adafruit_LIS3MDL.h"

struct magVector {
  int16_t x;
  int16_t y;
  int16_t z;
};


class Magnetometer {
public:
  Magnetometer(uint8_t i2c_addr);
  void init();

  magVector getMagneticVector();

private:
  Adafruit_LIS3MDL lis3mdl;
  const uint8_t addr;
  
};

#endif

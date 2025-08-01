#ifndef CLAW_H
#define CLAW_H

#include <Arduino.h>
#include "low_level/servodriver.h"
#include "include/low_level/magnetometer.h"
#include "include/low_level/rangefinder.h"

class Claw {
public:
  Claw(uint8_t m_servoPin);
  
  void init(void);
  
  void open(void);
  
  void close(void);

  int16_t getRangeFinderValue(void);

  inline bool getRangeFinderDataReady() {
    return rangeFinder.getDataReady();
  }

  magVector getMagnotometerValues(void);

private:
  Servo servo;
  Magnetometer magnetometer;
  RangeFinder rangeFinder;
};

#endif

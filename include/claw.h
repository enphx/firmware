#ifndef CLAW_H
#define CLAW_H

#include <Arduino.h>
#include "low_level/servodriver.h"

struct MagnetometerReadings {
  float x;
  float y;
  float z;
};

class Claw {
public:
  Claw(uint8_t m_leftServiPin, uint8_t m_rightServoPin);
  
  void init(void);
  
  void open(void);
  
  void close(void);

  float getRangeFinderValue(void);

  MagnetometerReadings getMagnotometerValues(void);

private:
  Servo leftServo, rightServo;
};

#endif

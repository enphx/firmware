#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>

class Servo {
public:
  Servo(uint8_t m_pwmPin, float m_minDutyCycle, float m_maxDutyCycle);
  void init(void);

  void setAngle(float angle);
private:
  uint16_t pwmPin;
  float minDutyCycle, maxDutyCycle;
};

#endif

#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>

#define SERVO_MIN_DUTY_CYCLE 0.02
#define SERVO_MIN_DUTY_CYCLE 0.12

class Servo {
public:
  Servo(uint8_t m_pwmPin, float m_minAngle, float m_maxAngle);
  void init(void);

  void setAngle(float angle);
private:
  uint16_t pwmPin;
  float minAngle, maxAngle;
};

#endif

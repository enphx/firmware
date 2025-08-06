#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>

class Servo {
public:
  Servo(uint8_t m_pwmPin, double m_minDutyCycle, double m_maxDutyCycle);
  void init(void);

  void setAngle(double angle);

  void flaccid(void);
private:
  uint16_t pwmPin;
  double minDutyCycle, maxDutyCycle;
};

#endif

#include "low_level/servodriver.h"
#include <Arduino.h>

Servo::Servo(uint8_t m_pwmPin, float m_minDutyCycle, float m_maxDutyCycle) {
  pwmPin = m_pwmPin;
  minDutyCycle = m_minDutyCycle * 0.01;
  maxDutyCycle = m_maxDutyCycle * 0.01;
}

void Servo::init(void) { ledcAttach(pwmPin, 50, 12); }

void Servo::setAngle(float angle) {
  angle = angle > 180 ? 180 : angle;
  angle = angle < 0 ? 0 : angle;

  float dutyCycle =
      minDutyCycle + (angle / 180.0) * (maxDutyCycle - minDutyCycle); 


  ledcWrite(pwmPin, (uint32_t)(dutyCycle * 4095));
}

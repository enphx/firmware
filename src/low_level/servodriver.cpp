#include "low_level/servodriver.h"
#include <Arduino.h>

Servo::Servo(uint8_t m_pwmPin, float m_minAngle, float m_maxAngle) {
  pwmPin = m_pwmPin;
  minAngle = m_minAngle;
  maxAngle = m_maxAngle;
}

void Servo::init(void) { ledcAttach(pwmPin, 50, 12); }

void Servo::setAngle(float angle) {
  if (angle > maxAngle) {
    angle = maxAngle;
  } else if (angle < minAngle) {
    angle = minAngle;
  }
  float dutyCycle = SERVO_MIN_DUTY_CYCLE + angle / 180 * 0.1 + 0.02;

  ledcWrite(pwmPin, (uint32_t) (dutyCycle * 4095));
}

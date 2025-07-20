#include "claw.h"
#include "constants.h"

Claw::Claw(uint8_t m_leftServoPin, uint8_t m_rightServoPin)
    : leftServo(m_leftServoPin, CLAW_MIN_PWM, CLAW_MAX_PWM),
      rightServo(m_rightServoPin, CLAW_MIN_PWM, CLAW_MAX_PWM) {}

void Claw::init(void) {
  leftServo.init();
  rightServo.init();
}

void Claw::open(void) {
  leftServo.setAngle(LEFT_CLAW_OPEN);
  rightServo.setAngle(RIGHT_CLAW_OPEN);
}

void Claw::close(void) {
  leftServo.setAngle(LEFT_CLAW_CLOSED);
  rightServo.setAngle(RIGHT_CLAW_CLOSED);
}

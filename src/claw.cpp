#include "claw.h"
#include "constants.h"
#include "Adafruit_LIS3MDL.h"
#include "include/low_level/io.h"
#include "low_level/magnetometer.h"


static const char* TAG = "CLAW";

Adafruit_LIS3MDL claw_magnetometer;

Claw::Claw(uint8_t m_leftServoPin, uint8_t m_rightServoPin)
    : leftServo(m_leftServoPin, CLAW_MIN_PWM, CLAW_MAX_PWM),
      rightServo(m_rightServoPin, CLAW_MIN_PWM, CLAW_MAX_PWM),
      magnetometer(I2C_CLAW_MAG_ADDR),
      rangeFinder(I2C_RANGEFINDER_ADDR) {}

void Claw::init(void) {
  ESP_LOGI(TAG, "init...");
  magnetometer.init();
  rangeFinder.init();
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

float Claw::getRangeFinderValue() {
  return rangeFinder.getDistance();
}

magVector Claw::getMagnotometerValues() {
  return magnetometer.getMagneticVector();
}

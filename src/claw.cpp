#include "claw.h"
#include "constants.h"
#include "Adafruit_LIS3MDL.h"
#include "include/low_level/io.h"
#include "low_level/magnetometer.h"


static const char* TAG = "CLAW";

Adafruit_LIS3MDL claw_magnetometer;

Claw::Claw(uint8_t m_servoPin) :
      servo(m_servoPin, CLAW_MIN_PWM, CLAW_MAX_PWM),
      magnetometer(I2C_CLAW_MAG_ADDR),
      rangeFinder(I2C_RANGEFINDER_ADDR) {}

void Claw::init(void) {
  ESP_LOGI(TAG, "init...");
  // rangeFinder.init();
  servo.init();
}

void Claw::open(void) {
  servo.setAngle(CLAW_OPEN);
}

void Claw::close(void) {
  servo.setAngle(CLAW_CLOSED);
}

int16_t Claw::getRangeFinderValue() {
  return rangeFinder.getDistance();
}

magVector Claw::getMagnotometerValues() {
  return magnetometer.getMagneticVector();
}

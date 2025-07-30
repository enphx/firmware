#include "include/low_level.h"
#include "Wire.h"
#include "core0.h"
#include "esp_log.h"
#include "include/low_level/io.h"
#include "include/serial/serial_comms.h"
#include <Arduino.h>

static const char *TAG = "LOW LEVEL";

ShiftRegister *shiftReg;

#define MOTOR_TICKS_PER_REV

EncoderMotor *leftMotor;
EncoderMotor *rightMotor;
PotentiometerMotor *shoulderMotor;
StepperMotor *asimuthStepper;
Servo *elbowServo;

void low_levelAssignMotors(EncoderMotor *m_leftMotor,
                           EncoderMotor *m_rightMotor,
                           PotentiometerMotor *m_shoulderMotor,
                           StepperMotor *m_asimuthMotor, Servo *m_elbowServo) {
  leftMotor = m_leftMotor;
  rightMotor = m_rightMotor;
  shoulderMotor = m_shoulderMotor;
  asimuthStepper = m_asimuthMotor;
  elbowServo = m_elbowServo;
}

void low_levelAssignLowestLevelObjects(ShiftRegister *m_shiftRegister) {
  shiftReg = m_shiftRegister;
}

void low_level_init() {
  ESP_LOGI(TAG, "init...");
  core0_init();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  shiftReg->init();
  leftMotor->init();
  rightMotor->init();
  shoulderMotor->init();
  asimuthStepper->init();
  elbowServo->init();
}

void set_speed(float speed) {
  leftMotor->setSpeed(speed);
  rightMotor->setSpeed(speed);
}

void print_adc_vals() {
  ESP_LOGI(
      TAG, "0: %hu, 1: %hu, 2: %hu, 3: %hu, 4: %hu, 5: %hu, 6: %hu, 7: %hu,",
      get_convolved_value(0), get_convolved_value(1), get_convolved_value(2),
      get_convolved_value(3), get_convolved_value(4), get_convolved_value(5),
      get_convolved_value(6), get_convolved_value(7));
}

void low_level_update() {
  // Now calling motor updates from drivebase...
  // rightMotor->update();
  // leftMotor->update();
  shoulderMotor->update();
}

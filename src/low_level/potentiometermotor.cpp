#include "low_level/potentiometermotor.h"
#include "constants.h"
#include "low_level/core0.h"

PotentiometerMotor::PotentiometerMotor(ShiftRegister *m_shiftRegister,
                                       const bool m_backwards,
                                       const uint8_t m_pwmPin,
                                       const uint8_t m_adcChannel,
                                       const uint8_t m_directionBit,
                                       const float m_Kp, const float m_Ki,
                                       const float m_Kd, const char m_ID)
    : shiftRegister(m_shiftRegister), pwmPin(m_pwmPin),
      adcChannel(m_adcChannel), directionBit(m_directionBit),
      positionPID(m_Kp, m_Ki, m_Kd, 3), ID(m_ID) {

  if (m_backwards) {
    backwards = -1;
  } else {
    backwards = 1;
  }
}
void PotentiometerMotor::setPID(float m_kP, float m_kI, float m_kD, float m_maxCumulativeError = 3) {
  positionPID.updateConstants(m_kP, m_kI, m_kD, m_maxCumulativeError);
}
void PotentiometerMotor::init(void) {
  pinMode(pwmPin, OUTPUT);

  ledcAttach(pwmPin, PWM_FREQUENCY, 12);
  timeLastUpdated = micros();
}

void PotentiometerMotor::update(void) {
  uint16_t potentiometerReading = get_convolved_value(adcChannel);

  currentAngle = (3450 - potentiometerReading) * DEGREES_PER_TICK_SHOULDER;

  float power = positionPID.update(currentAngle);

  if (power >= 0) {
    setPWM(power, 1);
  } else {
    setPWM(-power, 0);
  }
}

void PotentiometerMotor::calibrate(void) {}

void PotentiometerMotor::setAngle(float angle) { targetAngle = angle; 
  positionPID.setTargetValue(targetAngle);
}


float PotentiometerMotor::getAngle(void) { return currentAngle; }

void PotentiometerMotor::setPWM(float dutyCycle, uint8_t m_direction) {
  if (direction != m_direction) {
    ledcWrite(pwmPin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  direction = m_direction;
  shiftRegister->setBit(direction, directionBit);

  dutyCycle = dutyCycle > 1 ? 1 : dutyCycle;

  ledcWrite(pwmPin, (uint32_t)(dutyCycle * 4095));
}


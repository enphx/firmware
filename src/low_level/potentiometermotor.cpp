#include "low_level/potentiometermotor.h"
#include "constants.h"
#include "low_level/core0.h"

PotentiometerMotor::PotentiometerMotor(
    ShiftRegister *m_shiftRegister, const bool m_backwards,
    const uint8_t m_pwmPin, const uint8_t m_adcChannel,
    const uint8_t m_directionBit, const float m_minAnlge,
    const float m_maxAngle, const float m_Kp, const float m_Ki,
    const float m_Kd, const char m_ID)
    : shiftRegister(m_shiftRegister), pwmPin(m_pwmPin),
      adcChannel(m_adcChannel), directionBit(m_directionBit),
      minAngle(m_minAnlge), maxAngle(m_maxAngle), Kp(m_Kp), Ki(m_Ki), Kd(m_Kd),
      ID(m_ID) {

  currentAngle = minAngle;

  if (m_backwards) {
    backwards = -1;
  } else {
    backwards = 1;
  }
}

void PotentiometerMotor::init(void) {
  pinMode(pwmPin, OUTPUT);

  ledcAttach(pwmPin, PWM_FREQUENCY, 12);
  timeLastUpdated = micros();
}

void PotentiometerMotor::update(void) {
  deltaT = micros() - timeLastUpdated;
  if (deltaT == 0) {
    return;
  }
  timeLastUpdated += deltaT;
  previousError = error;
  uint16_t potentiometerReading = get_convolved_value(adcChannel);
  currentAngle = potentiometerReading / VOLTS_PER_DEGREE_INT_POT;
  error = targetAngle - currentAngle;
  cumulativeError += error * deltaT;

  float maxCumulativeError = 20;
  cumulativeError = cumulativeError > maxCumulativeError ? maxCumulativeError
                                                         : cumulativeError;
  cumulativeError = cumulativeError < -maxCumulativeError ? -maxCumulativeError
                                                          : cumulativeError;

  float power = calculatePID() * backwards;

  if (power >= 0) {
    setPWM(power, 1);
  } else {
    setPWM(-power, 0);
  }
}

void PotentiometerMotor::calibrate(void) {}

void PotentiometerMotor::setAngle(float angle) { targetAngle = angle; }

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

float PotentiometerMotor::calculatePID(void) {
  float P = Kp * error;
  float I = Ki * cumulativeError;
  float D = Kd * (error - previousError) / (deltaT);
  return P + I + D;
}

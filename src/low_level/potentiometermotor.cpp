#include "low_level/potentiometermotor.h"
#include "constants.h"
#include "low_level/core0.h"

PotentiometerMotor::PotentiometerMotor(ShiftRegister *m_shiftRegister,
                                       const bool m_backwards,
                                       const uint8_t m_pwmPin,
                                       const uint8_t m_adcChannel,
                                       const uint8_t m_directionBit,
                                       const double m_Kp, const double m_Ki,
                                       const double m_Kd, const char m_ID)
    : shiftRegister(m_shiftRegister), pwmPin(m_pwmPin),
      adcChannel(m_adcChannel), directionBit(m_directionBit), Kp(m_Kp),
      Ki(m_Ki), Kd(m_Kd), ID(m_ID) {

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

  currentAngle = (3450 - potentiometerReading) * DEGREES_PER_TICK_SHOULDER;
  error = targetAngle - currentAngle;
  cumulativeError += error * deltaT * 0.000001;

  cumulativeError = cumulativeError > maxCE ? maxCE
                                                         : cumulativeError;
  cumulativeError = cumulativeError < -maxCE ? -maxCE
                                                          : cumulativeError;

  double power = calculatePID() * backwards;

  if (power >= 0) {
    setPWM(power, 1);
  } else {
    setPWM(-power, 0);
  }
}


void PotentiometerMotor::setAngle(double angle) { targetAngle = angle; }

double PotentiometerMotor::getAngle(void) { return currentAngle; }

void PotentiometerMotor::setPWM(double dutyCycle, uint8_t m_direction) {
  if (direction != m_direction) {
    ledcWrite(pwmPin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  direction = m_direction;
  shiftRegister->setBit(direction, directionBit);

  dutyCycle = dutyCycle > 1 ? 1 : dutyCycle;

  ledcWrite(pwmPin, (uint32_t)(dutyCycle * 4095));
}

double PotentiometerMotor::calculatePID(void) {
  P = Kp * error;
  I = Ki * cumulativeError;
  D = Kd * 1000.0f * (error - previousError) / (deltaT);
  return P + I + D;
}

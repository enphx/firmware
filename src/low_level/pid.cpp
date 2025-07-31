#include "low_level/pid.h"

PID::PID(float m_Kp, float m_Ki, float m_Kd, float m_cumulativeErrorMax) {
  Kp = m_Kp;
  Ki = m_Ki;
  Kd = m_Kd;
  cumulativeErrorMax = m_cumulativeErrorMax;
  timeLastUpdated = micros();
}

void PID::updateConstants(float m_Kp, float m_Ki, float m_Kd,
                          float m_cumulativeErrorMax) {
  Kp = m_Kp;
  Ki = m_Ki;
  Kd = m_Kd;
  cumulativeErrorMax = m_cumulativeErrorMax;
}

float PID::update(float m_currentValue) {
  deltaT = micros() - timeLastUpdated;
  timeLastUpdated += deltaT;
  currentValue = m_currentValue;
  previousError = error;
  error = targetValue - currentValue;
  cumulativeError += error * deltaT * 0.1;

  if (cumulativeErrorMax != -1) {

    cumulativeError = cumulativeError > cumulativeErrorMax ? cumulativeErrorMax
                                                           : cumulativeError;
    cumulativeError = cumulativeError < -cumulativeErrorMax
                          ? -cumulativeErrorMax
                          : cumulativeError;
  }

  P = error * Kp;
  I = Ki * cumulativeError;
  D = Kd * (error - previousError) / deltaT * 1000000.0f;

  return P + I + D;
}

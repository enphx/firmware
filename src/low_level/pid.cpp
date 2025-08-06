#include "low_level/pid.h"

PID::PID(double m_Kp, double m_Ki, double m_Kd, double m_cumulativeErrorMax) {
  Kp = m_Kp;
  Ki = m_Ki;
  Kd = m_Kd;
  cumulativeErrorMax = m_cumulativeErrorMax;
  timeLastUpdated = micros();
}

void PID::updateConstants(double m_Kp, double m_Ki, double m_Kd,
                          double m_cumulativeErrorMax) {
  Kp = m_Kp;
  Ki = m_Ki;
  Kd = m_Kd;
  cumulativeErrorMax = m_cumulativeErrorMax;
}

double PID::update(double m_currentValue) {
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

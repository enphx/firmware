#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID {
public:
  /**
   *  @param set cumulativeErrorMax to -1 for no clamp
   */
  PID(float m_Kp, float m_Ki, float m_Kd, float m_cumulativeErrorMax = -1);
  void updateConstants(float m_Kp, float m_Ki, float m_Kd,
                       float m_cumulativeErrorMax);
  float update(float m_currentValue);
  inline void setTargetValue(float m_target) { targetValue = m_target; }
  inline float getError(void) { return error; }
  inline float getP(void) { return P; }
  inline float getI(void) { return I; }
  inline float getD(void) { return D; }

private:
  float currentValue, targetValue;
  float cumulativeError, previousError, error;
  float cumulativeErrorMax = -1;
  float Kp, Ki, Kd;
  float P, I, D;
  uint32_t timeLastUpdated, deltaT;
};

#endif

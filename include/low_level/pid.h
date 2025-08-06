#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID {
public:
  /**
   *  @param dont give input for cumulativeErrorMax for it to be uncapped
   */
  PID(double m_Kp, double m_Ki, double m_Kd, double m_cumulativeErrorMax = -1);
  void updateConstants(double m_Kp, double m_Ki, double m_Kd,
                       double m_cumulativeErrorMax = -1);
  double update(double m_currentValue);
  inline void setTargetValue(double m_target) { targetValue = m_target; }
  inline double getError(void) { return error; }
  inline double getTargetValue() {return targetValue; }
  inline double getP(void) { return P; }
  inline double getI(void) { return I; }
  inline double getD(void) { return D; }


  inline void getPIDVals(double *err, double *setpoint, double *p_out, double *i_out, double *d_out) {
    *err = getError();
    *setpoint = getTargetValue();
    *p_out = getP();
    *i_out = getI();
    *d_out = getD();
  }

private:
  double currentValue, targetValue;
  double cumulativeError, previousError, error;
  double cumulativeErrorMax = -1;
  double Kp, Ki, Kd;
  double P, I, D;
  uint32_t timeLastUpdated, deltaT;
};

#endif

#ifndef POTENTIOMETER_MOTOR_H
#define POTENTIOMETER_MOTOR_H
#include "low_level/core0/adc.h"
#include "low_level/shiftregister.h"
#include <Arduino.h>

class PotentiometerMotor {
public:
  PotentiometerMotor(ShiftRegister *m_shiftRegister, const bool m_backwards,
                     const uint8_t m_pwmPin, const uint8_t m_adcChannel,
                     const uint8_t m_directionBit, const double m_Kp,
                     const double m_Ki, const double m_Kd, const char m_ID);

  void init(void);

  void update(void);

  void setAngle(double angle);

  double getAngle(void);

  inline void setPID(double m_Kp, double m_Ki, double m_Kd) {
    Kp = m_Kp;
    Ki = m_Kp;
    Kd = m_Kp;
  }

  inline double getError() {return error;}
  inline double getSetPoint() {return targetAngle;}
  inline double getP() {return P;}
  inline double getI() {return I;}
  inline double getD() {return D;}

  inline void setMaxCE(double max_ce) {maxCE = max_ce;}

private:
  double calculatePID(void);
  void setPWM(double dutyCycle, uint8_t m_direction);

  const char ID;

  const uint8_t pwmPin, directionBit, adcChannel;

  uint16_t minPotentiometerReading, maxPotentiometerReading;

  ShiftRegister *const shiftRegister;
  int backwards = 1;
  double currentAngle, targetAngle;
  double error = 0, previousError, cumulativeError = 0;
  uint64_t timeLastUpdated;
  uint32_t deltaT;
  uint8_t direction = 0;
  double Kp, Ki, Kd;
  double maxCE = 0;
  double P, I, D = 0.0;
};

#endif

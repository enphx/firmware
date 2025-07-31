#ifndef POTENTIOMETER_MOTOR_H
#define POTENTIOMETER_MOTOR_H
#include "low_level/core0/adc.h"
#include "low_level/shiftregister.h"
#include <Arduino.h>

class PotentiometerMotor {
public:
  PotentiometerMotor(ShiftRegister *m_shiftRegister, const bool m_backwards,
                     const uint8_t m_pwmPin, const uint8_t m_adcChannel,
                     const uint8_t m_directionBit, const float m_Kp,
                     const float m_Ki, const float m_Kd, const char m_ID);

  void init(void);

  void update(void);

  void calibrate(void);

  void setAngle(float angle);

  float getAngle(void);

  inline void setPID(float m_Kp, float m_Ki, float m_Kd) {
    Kp = m_Kp;
    Ki = m_Kp;
    Kd = m_Kp;
  }

  inline float getError() {return error;}
  inline float getSetPoint() {return targetAngle;}
  inline float getP() {return P;}
  inline float getI() {return I;}
  inline float getD() {return D;}

private:
  float calculatePID(void);
  void setPWM(float dutyCycle, uint8_t m_direction);

  const char ID;

  const uint8_t pwmPin, directionBit, adcChannel;

  uint16_t minPotentiometerReading, maxPotentiometerReading;

  ShiftRegister *const shiftRegister;
  int backwards = 1;
  float currentAngle, targetAngle;
  float error = 0, previousError, cumulativeError = 0;
  uint64_t timeLastUpdated;
  uint32_t deltaT;
  uint8_t direction = 0;
  float Kp, Ki, Kd;
  float P, I, D = 0.0;
};

#endif

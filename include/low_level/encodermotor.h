#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H
#include "include/low_level/shiftregister.h"
#include "low_level/pid.h"
#include <Arduino.h>

class EncoderMotor {
public:
  EncoderMotor(ShiftRegister *m_shiftReg, const bool m_backwards,
               const uint8_t m_encoderPin1, const uint8_t m_encoderPin2,
               const uint8_t m_pwmPin, const uint8_t m_dirBit,
               const int m_ticksPerRev, const float m_kP, const float m_kI,
               const float m_kD, const char ID);

  void init();
  
  int32_t update(void);

  uint8_t getDirection(void);

  uint8_t getDirectionBit(void);

  void setSpeed(float speed);

  float getCurrentSpeed(void);

  long getTickCount(void);

private:
  void setPID(float m_kP, float m_kI, float m_kD, float m_maxCumulativeError);

  void setPWM(float dutycycle, uint8_t m_direction);
  float calculatePID(void);
  static void genericEncoderHandler(void *arg);
  void handleEncoder();

  ShiftRegister *const shiftReg;

  const uint8_t encoderPin1, encoderPin2, pwmPin, dirBit;
  const int ticksPerRev;

  const char ID;

  PID velocityPID;

  float kP = 0, kD = 0, kI = 0;
  float targetSpeed = 0, currentSpeed = 0;
  volatile uint8_t previousState = 0;
  volatile int32_t tickCount = 0;
  int32_t previousTickCount = 0;
  uint64_t timeLastUpdated;
  uint32_t deltaT;
  uint8_t direction = 0;
  int32_t currentTickCount = 0;
  float previousError = 0;
  float error = 0;
  float cumulativeError = 0;
  int backwards = 1;
};

#endif

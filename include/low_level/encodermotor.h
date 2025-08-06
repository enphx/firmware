#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H
#include <Arduino.h>
#include "include/low_level/shiftregister.h"

void encoderHandler(void *arg);

struct EncoderData {
  const volatile uint8_t pin1;
  const volatile uint8_t pin2;
  volatile uint8_t prev_state;
  volatile int32_t tick_count;
};

class EncoderMotor {
public:
  EncoderMotor(ShiftRegister* m_shiftReg,
               const bool m_backwards,
               const uint8_t m_encoderPin1,
               const uint8_t m_encoderPin2,
               const uint8_t m_pwmPin,
               const uint8_t m_dirBit,
               const int m_ticksPerRev,
               const double m_kP,
               const double m_kI,
               const double m_kD,
               const char ID
              );

  void init();
  

  // Returns change in ticks for odometry tracking :D
  int32_t update(void);

  uint8_t getDirection(void);

  uint8_t getDirectionBit(void);

  void setSpeed(double speed);

  double getCurrentSpeed(void);

  long getTickCount(void);

  void setPID(double m_kP, double m_kI, double m_kD, double max_cum_error = -1);

  inline double getError() {return error;}
  inline double getSetPoint() {return targetSpeed;}
  inline double getP() {return P;}
  inline double getI() {return I;}
  inline double getD() {return D;}

private:

  void setPWM(double dutycycle, uint8_t m_direction);
  double calculatePID(void); 

  ShiftRegister* const shiftReg;

  volatile EncoderData encoder_data;

  const uint8_t  pwmPin, dirBit;
  const int ticksPerRev;

  const char ID;

  double kP = 0, kD = 0, kI = 0;
  double targetSpeed = 0, currentSpeed = 0;
  int32_t previousTickCount = 0;
  uint64_t timeLastUpdated;
  uint32_t deltaT;
  uint8_t direction = 0;
  int32_t currentTickCount = 0;
  double previousError = 0;
  double error = 0;
  double cumulativeError = 0;
  double maxCumError = 30000.0;
  int backwards = 1;
  uint32_t lastSpeedReadingTime = 0;
  int32_t lastSpeedReadingTicks = 0;

  double P;
  double I;
  double D;

};



#endif

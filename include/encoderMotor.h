#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H
#include <Arduino.h>

class EncoderMotor {
public:
  EncoderMotor();
  void begin(const uint8_t m_encoderPin1, const uint8_t m_encoderPin2,
               const uint8_t m_pwmPin, const uint8_t m_pwmChannel,
               const uint8_t m_direction_bit, const int m_ticksPerRev);

  void update(void);

  void setBackwards(boolean backwards);

  uint8_t getDirection(void);

  uint8_t getDirectionBit(void);

  void setSpeed(float speed);

  void setPID(float m_kP, float m_kD, float m_kI);

  float getCurrentSpeed(void);

  long getTickCount(void);

  void tapeFollowingCorrection(float correction);

private:
  static constexpr float wheelDiameter = 0.07;
  static constexpr int pwmFrequency = 1000;

  void setPWM(float dutycycle, uint8_t m_direction);
  float calculatePID(void); 
  static void genericEncoderHandler(void *arg);
  void handleEncoder();

  uint8_t encoderPin1, encoderPin2, pwmPin, pwmChannel, direction_bit;
  int ticksPerRev;

  float kP = 0, kD = 0, kI = 0;
  float targetSpeed = 0, currentSpeed = 0;
  volatile long tickCount = 0;
  volatile uint8_t previousState = 0;
  uint64_t timeLastUpdated;
  uint32_t deltaT;
  uint8_t direction = 0;
  long previousTickCount = 0;
  long currentTickCount = 0;
  float previousError = 0;
  float error = 0;
  float cumulativeError = 0;
  int backwards = 1;
  float tapeFollowingCorrectionPower = 0.0f;

};

#endif

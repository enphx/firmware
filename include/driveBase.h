#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include <Arduino.h>

enum tapeState {
  Difference,
  Inversion,
  OutOfBounds,
  Left,
  Right,
};

class EncoderMotor {};

class TapeFollowingSenosr {};

class DriveBase {
public:
  DriveBase(void);

  void begin();

  void update(void);

  void setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd);

  void setBaseSpeed(float speed);

  const tapeState getTapeState();

  const tapeState getSide();

private:
  float calculateError();
  void findTape();
  EncoderMotor *leftMotor, *rightMotor;
  TapeFollowingSenosr *tapeFollowingSensor;

  float Kp = 0.0f, Kd = 0.0f, Ki = 0.0f;
  tapeState Distance, Side;
  int error, previousError;
  int lastDifferenceReadingLeft, lastDifferenceReadingRight;
  float baseSpeed;
  uint64_t timeLastUpdated;
  uint32_t deltaT;
};

#endif

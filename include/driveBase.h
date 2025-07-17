#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include "low_level/tapefollowingsensor.h"
#include <Arduino.h>

enum tapeState {
  Difference,
  Inversion,
  OutOfBounds,
  Left,
  Right,
};

class EncoderMotor {
  void setSpeed(float speed);
};

class DriveBase {
public:
  DriveBase(EncoderMotor *m_leftMotor, EncoderMotor *m_rightMotor,
            TapeFollowingSensor *m_tapeFollowingSensor);

  void update(void);

  void setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd);

  void setBaseSpeed(float speed);

private:
  float calculateCorrection();
  void findTape();
  EncoderMotor *leftMotor, *rightMotor;
  TapeFollowingSensor *tapeFollowingSensor;

  float Kp = 0.0f, Kd = 0.0f, Ki = 0.0f;
  int error = 0, previousError;
  tapeState previousTapeState, previousSide;
  float baseSpeed;
  uint64_t timeLastUpdated;
  uint32_t deltaT;
};

#endif

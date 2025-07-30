#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include "low_level/tapefollowingsensor.h"
#include "low_level/encodermotor.h"
#include <Arduino.h>

#define TICKS_PER_REVOLUTION 524
// In meters:
#define WHEEL_RADIUS 0.037
#define PI 3.141592653589793238462643383279

struct RobotPosition {
  float x;
  float y;
  float theta;
};

class DriveBase {
public:
  DriveBase(EncoderMotor *m_leftMotor, EncoderMotor *m_rightMotor,
            TapeFollowingSensor *m_tapeFollowingSensor);

  void update(void);

  void setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd);

  void setBaseSpeed(float speed);

  void followLine(bool m_lineFollow);

  inline int getTapeFollowingError() {
    return tapeFollowingSensor->getError();
  }

  // Odometry tech

  inline void setOdometry(float m_x, float m_y, float m_theta) {
    x = m_x;
    y = m_y;
    theta = m_theta;
  }

  inline void setOdometry(RobotPosition pos) {
    x = pos.x;
    y = pos.y;
    theta = pos.theta;
  }
  
  inline void resetOdometry() {
    setOdometry(0, 0, 0);
  }

  inline RobotPosition getPosition() {
    return {x, y, theta};
  }



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
  bool lineFollow = true;

  // Odometry tech
  float x = 0;
  float y = 0;

  float theta = 0;

  constexpr static float DISTANCE_PER_TICK = (WHEEL_RADIUS * 2 * PI / TICKS_PER_REVOLUTION);
};

#endif

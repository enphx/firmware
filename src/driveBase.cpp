#include "driveBase.h"
#include "low_level/fastfunctions.h"

static const char *TAG = "DRIVEBASE";

#define DRIVE_BASE_LENGTH 0.26

DriveBase::DriveBase(EncoderMotor *m_leftMotor, EncoderMotor *m_rightMotor,
                     TapeFollowingSensor *m_tapeFollowingSensor, float m_Kp,
                     float m_Ki, float m_Kd)
    : tapeFollowingPID(m_Kp, m_Ki, m_Kd) {
  leftMotor = m_leftMotor;
  rightMotor = m_rightMotor;
  tapeFollowingSensor = m_tapeFollowingSensor;
  tapeFollowingPID.setTargetValue(0);
}

void DriveBase::setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd, float m_cumulativeErrorMax) {
  tapeFollowingPID.updateConstants(m_Kp, m_Ki, m_Kd, m_cumulativeErrorMax);
}

void DriveBase::followLine(bool m_lineFollow) { lineFollow = m_lineFollow; }

void DriveBase::update(void) {

  int32_t deltaTicksL = leftMotor->update();
  int32_t deltaTicksR = rightMotor->update();
  float distL = DISTANCE_PER_TICK * deltaTicksL;
  float distR = DISTANCE_PER_TICK * deltaTicksR;

  if (deltaTicksL == deltaTicksR) {
    x += distL * fast_cos(theta);
    y += distL * fast_sin(theta);
  } else {
    float deltaTheta = (distR - distL / DRIVE_BASE_LENGTH);
    float turnRadius = (distR + distL) / (2 * deltaTheta);
    float arcLength = (distR + distL) / 2;

    x += arcLength * fast_cos(theta + deltaTheta / 2);
    y += arcLength * fast_sin(theta + deltaTheta / 2);
    theta += deltaTheta;
  }

  tapeFollowingSensor->update();

  if (lineFollow &&
      tapeFollowingSensor->getTapeState() == tapeState::OutOfBounds) {
    findTape();
    return;
  }

  error = tapeFollowingSensor->getError();

  float correction = tapeFollowingPID.update(-error);

  if (lineFollow) {
    leftMotor->setSpeed(baseSpeed + correction);
    rightMotor->setSpeed(baseSpeed - correction);
  } else {
    leftMotor->setSpeed(baseSpeed);
    rightMotor->setSpeed(baseSpeed);
  }

  previousTapeState = tapeFollowingSensor->getTapeState();
  previousSide = tapeFollowingSensor->getSide();
}
void DriveBase::setBaseSpeed(float speed) { baseSpeed = speed; }

void DriveBase::findTape(void) {
  if (tapeFollowingSensor->getSide() == tapeState::Left) {
    leftMotor->setSpeed(baseSpeed);
    rightMotor->setSpeed(0.2 * baseSpeed);
    return;
  }
  rightMotor->setSpeed(baseSpeed);
  leftMotor->setSpeed(0.2 * baseSpeed);
}

#include "driveBase.h"

static const char *TAG = "DRIVEBASE";

// TODO: Write DriveBase Control
DriveBase::DriveBase(EncoderMotor *m_leftMotor, EncoderMotor *m_rightMotor,
                     TapeFollowingSensor *m_tapeFollowingSensor) {
  leftMotor = m_leftMotor;
  rightMotor = m_rightMotor;
  tapeFollowingSensor = m_tapeFollowingSensor;
}

void DriveBase::setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd) {
  Kp = m_Kp;
  Ki = m_Ki;
  Kd = m_Kd;
}

void DriveBase::followLine(bool m_lineFollow) { lineFollow = m_lineFollow; }

void DriveBase::update(void) {
  deltaT = micros() - timeLastUpdated;
  timeLastUpdated += deltaT;
  if (!lineFollow) {
    leftMotor->setSpeed(baseSpeed);
    rightMotor->setSpeed(baseSpeed);
  }
  previousError = error;

  if (tapeFollowingSensor->getTapeState() == tapeState::OutOfBounds) {
    findTape();
    return;
  }

  error = tapeFollowingSensor->getError();

  if (previousTapeState == tapeState::OutOfBounds) {
    previousError = error;
  }

  leftMotor->setSpeed(baseSpeed + calculateCorrection());
  rightMotor->setSpeed(baseSpeed - calculateCorrection());

  previousTapeState = tapeFollowingSensor->getTapeState();
  previousSide = tapeFollowingSensor->getSide();
}
void DriveBase::setBaseSpeed(float speed) { baseSpeed = speed; }

float DriveBase::calculateCorrection(void) {
  float proportionalTerm = Kp * 0.001f;
  float derrivativeTerm = Kd * 0.001f * (error - previousError) / float(deltaT);
  return (proportionalTerm + derrivativeTerm) * baseSpeed;
}

void DriveBase::findTape(void) {
  if (tapeFollowingSensor->getSide() == tapeState::Left) {
    leftMotor->setSpeed(baseSpeed);
    rightMotor->setSpeed(0.2 * baseSpeed);
    return;
  }
  rightMotor->setSpeed(baseSpeed);
  leftMotor->setSpeed(0.2 * baseSpeed);
}

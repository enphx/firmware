#include "driveBase.h"

// TODO: Write DriveBase Control
DriveBase::DriveBase(void) {}

void DriveBase::begin() {
  timeLastUpdated = micros();
}

void DriveBase::setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd) {
  Kp = m_Kp;
  Ki = m_Ki;
  Kd = m_Kd;
}

void DriveBase::update(void) {
  deltaT = micros() - timeLastUpdated;
  timeLastUpdated += deltaT;

}
void setBaseSpeed(float speed) {}

const tapeState DriveBase::getTapeState(void) {
  return Distance;
}

const tapeState  DriveBase::getSide(void) {
  return Side;
}

float DriveBase::calculateError(void) {
  return 0.0f;
}

void DriveBase::findTape(void) {}

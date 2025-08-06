#include "driveBase.h"
#include "low_level/fastfunctions.h"
#include "include/serial/serial_comms.h"

static const char *TAG = "DRIVEBASE";

// Distance between wheels in meters.
#define DRIVE_BASE_LENGTH 0.26

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

  if (!leftMotor || !rightMotor || !tapeFollowingSensor) {
    ESP_LOGE(TAG, "Null pointer in DriveBase::update()");
    return;
  }

  
  // Update Odometry first thing.
  
  // DOUBLE CHECK WITH YUVRAJ THAT RUNNING UPDATE BEFORE IS CHILL!
  int32_t deltaTicksL = leftMotor->update();
  int32_t deltaTicksR = rightMotor->update();
  float distL = -DISTANCE_PER_TICK * deltaTicksL;
  float distR = DISTANCE_PER_TICK * deltaTicksR;
  float deltaTheta = 0;
  float arcLength;

  // ESP_LOGI(TAG, "distR: %f, distL: %f", distR, distL);

  if (deltaTicksL == deltaTicksR) {
    arcLength = distL;
    x += distL * fast_cos(theta);
    y += distL * fast_sin(theta);
  } else {
    deltaTheta = ((distR - distL) / DRIVE_BASE_LENGTH);
    // float turnRadius = (distR + distL) / (2 * deltaTheta);
    arcLength = (distR + distL)/2;
    x += arcLength * fast_cos(theta + deltaTheta/2);
    y += arcLength * fast_sin(theta + deltaTheta/2);
    theta += deltaTheta;
  }

  distanceTravelled += arcLength;
  

  // Update tape following shenanigans.
  
  tapeFollowingSensor->update();
  deltaT = micros() - timeLastUpdated;

  if (deltaT == 0 || Ki == 0.0) {
    ESP_LOGE(TAG, "Zero value for dividing...");
    return;
  }

  timeLastUpdated += deltaT;
  previousError = error;


  if (tapeFollowingSensor->getTapeState() == tapeState::OutOfBounds) {
    // findTape();
    // return;
    cumError += (tapeFollowingSensor->getSide() == tapeSide::Right ? -arcLength : arcLength);
  } else {
    cumError += (cumError > 0 ? -arcLength : arcLength);
  }

  if (tapeFollowingSensor->getSide() != previousSide) {
  }

  cumError = cumError >= 1.0/Ki ? 1.0/Ki : cumError;
  cumError = cumError <= -1.0/Ki ? -1.0/Ki : cumError;

  error = tapeFollowingSensor->getError();

  if (previousTapeState == tapeState::OutOfBounds) {
    previousError = error;
    if (tapeFollowingSensor->getTapeState() != tapeState::OutOfBounds) {
      cumError = cumError * 0.5;
    }
  }

  


  if (turning) {
    // ESP_LOGI(TAG, "Turning with direction %i and speed %f");
    leftMotor->setSpeed(baseSpeed * turningDirection);
    rightMotor->setSpeed(baseSpeed * -turningDirection);
  } else if (lineFollow) {
    float correction = calculateCorrection();
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

float DriveBase::calculateCorrection(void) {
  P = Kp * 0.001f * error;
  D = Kd * 0.001f * (error - previousError) / float(deltaT);
  I = Ki * cumError;
  float output = P + I + D;
  output = output >  1.0 ?  1.0 : output;
  output = output < -1.0 ? -1.0 : output;
  return output * baseSpeed;
}

void DriveBase::findTape(void) {
  if (tapeFollowingSensor->getSide() == tapeSide::Left) {
    leftMotor->setSpeed(baseSpeed);
    rightMotor->setSpeed(0.2 * baseSpeed);
    return;
  }
  rightMotor->setSpeed(baseSpeed);
  leftMotor->setSpeed(0.2 * baseSpeed);
}

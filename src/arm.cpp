#include "arm.h"
#include "low_level/fastfunctions.h"
#include "low_level/stepperdriver.h"

static const char *TAG = "ARM";

Arm::Arm(PotentiometerMotor *m_shoulderMotor, Servo *m_elbowServo,
         StepperMotor *m_asimuthStepper) {
  shoulderMotor = m_shoulderMotor;
  elbowServo = m_elbowServo;
  asimuthStepper = m_asimuthStepper;
}

void Arm::setArmPosition(float radius, float height, float theta) {

  height = height < 1 ? 1 : height;
  radius = radius > 16 ? 16 : radius;

  ArmAngles angles = calculateInverseKinematics(radius, height, theta);
  asimuthStepper->setAngle(angles.asimuthTheta);

  ESP_LOGI(TAG, "shoulderAngle: %f", angles.shoulderTheta);
  ESP_LOGI(TAG, "elbowAngle: %f", angles.elbowTheta);

  shoulderMotor->setAngle(angles.shoulderTheta * 180.0 / PI);
  elbowServo->setAngle(angles.elbowTheta * 180.0 / PI+10);
}

void Arm::setArmAngles(float asimuthTheta, float shoulderTheta, float elbowTheta) {
  asimuthStepper->setAngle(asimuthTheta);
  shoulderMotor->setAngle(shoulderTheta);
  elbowServo->setAngle(elbowTheta);
}


ArmAngles Arm::calculateInverseKinematics(float radius, float height,
                                          float theta) {
  ArmAngles angles;
  float adjacantElbow = radius * radius + (height - 7) * (height - 7) - 128;
  float hypotenusEblow = 128;
  angles.elbowTheta = acos_spl(adjacantElbow / hypotenusEblow);

  float oppositeShoulder = height - 7;
  float adjacantShoulder = radius;
  angles.shoulderTheta =
      atan_spl(oppositeShoulder / adjacantShoulder) + 0.5 * angles.elbowTheta;

  angles.asimuthTheta = theta;

  return angles;
}

float Arm::getTheta() {
  return asimuthStepper->getAngleAbsolute();
}

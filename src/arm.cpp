#include "arm.h"

Arm::Arm(PotentiometerMotor *m_shoulderMotor, Servo *m_elbowServo,
         StepperMotor *m_asimuthStepper) {
  shoulderMotor = m_shoulderMotor;
  elbowServo = m_elbowServo;
  asimuthStepper = m_asimuthStepper;
}

void Arm::setArmPosition(float radius, float height, float theta) {

  ArmAngles angles = calculateInverseKinematics(radius, height, theta);

  asimuthStepper->setAngle(angles.asimuthTheta);
  shoulderMotor->setAngle(angles.shoulderTheta);
  elbowServo->setAngle(angles.elbowTheta);
}

ArmAngles Arm::calculateInverseKinematics(float radius, float height, float theta) {
  ArmAngles angles;


  return angles;
}

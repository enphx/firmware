#include "arm.h"
#include "low_level/fastfunctions.h"

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
  float adjacantElbow = radius * radius + (height - 7) * (height - 7) - 128;
  float hypotenusEblow = 128;
  angles.elbowTheta = acos_spl(adjacantElbow / hypotenusEblow);

  float oppositeShoulder = height - 7;
  float adjacantShoulder = radius;
  angles.shoulderTheta = atan_spl(oppositeShoulder / adjacantShoulder) + 0.5 * angles.elbowTheta;

  angles.asimuthTheta = theta;


  return angles;
}

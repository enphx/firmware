#ifndef ARM_H
#define ARM_H
#include "low_level/potentiometermotor.h"
#include "low_level/servodriver.h"
#include "low_level/stepperdriver.h"
#include <Arduino.h>

struct ArmAngles {
  float asimuthTheta;
  float shoulderTheta;
  float elbowTheta;
};

struct ArmPosition {
  float height;
  float radius;
  float theta;
};

class Arm {
public:
  Arm(PotentiometerMotor *m_shoulderMotor, Servo *m_elbowServo,
      StepperMotor *m_asimuthStepper);

  void setArmPosition(float radius, float height, float theta);

  void setArmAngles(float asimuthTheta, float shoulderTheta, float elbowTheta);

  float getTheta();

  inline void calibrate(void) {
    asimuthStepper->calibrate();
  }

  inline ArmPosition getPosition(void) {
    return {targetHeight, targetRadius, targetTheta};
  }

private:
  ArmAngles calculateInverseKinematics(float radius, float height, float theta);

  float targetRadius, targetHeight, targetTheta;

  PotentiometerMotor *shoulderMotor;
  Servo *elbowServo;
  StepperMotor *asimuthStepper;
};

#endif

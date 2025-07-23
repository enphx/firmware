#ifndef ROBOT_H
#define ROBOT_H

#include "low_level/shiftregister.h"
#include "low_level/encodermotor.h"
#include "low_level/tapefollowingsensor.h"
#include "low_level/potentiometermotor.h"
#include "low_level/stepperdriver.h""
#include "driveBase.h"
#include "low_level/io.h"
#include "arm.h"
#include "claw.h"

class Robot {
public:
  Robot();
  void init(void);

  void update();

private:
  EncoderMotor leftMotor, rightMotor;
  PotentiometerMotor shoulderMotor;
  StepperMotor asimuthStepper;
  TapeFollowingSensor tapeFollowingSensor;
  DriveBase driveBase;
  ShiftRegister shiftRegister;
};

#endif

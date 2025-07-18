#ifndef ROBOT_H
#define ROBOT_H

#include "low_level/shiftregister.h"
#include "low_level/encodermotor.h"
#include "low_level/tapefollowingsensor.h"
#include "driveBase.h"
#include "low_level/io.h"

class Robot {
public:
  Robot();
  void init(void);

  void update();

private:
  EncoderMotor leftMotor, rightMotor;
  TapeFollowingSensor tapeFollowingSensor;
  DriveBase driveBase;
  ShiftRegister shiftRegister;
};

#endif

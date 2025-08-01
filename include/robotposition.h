#ifndef ROBOT_POS_H
#define ROBOT_POS_H

#include "include/driveBase.h"
#include "include/arm.h"

struct RobotPosition {
  DriveBasePosition driveBasePosition;
  ArmPosition armPosition;
};

#endif

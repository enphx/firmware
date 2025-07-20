#include "robot.h"
#include "low_level.h"
#include <Arduino.h>

Robot::Robot()
    : shiftRegister(),

      leftMotor(&shiftRegister, true, PIN_M1_ENC2, PIN_M1_ENC1, PIN_M1_PWM,
                BIT_M1_DIR, 44 * 21, 2, 0.05, 1, 'L'),
      rightMotor(&shiftRegister, false, PIN_M2_ENC2, PIN_M2_ENC1, PIN_M2_PWM,
                 BIT_M2_DIR, 44 * 21, 2, 0.05, 1, 'R'),

      tapeFollowingSensor(),
      driveBase(&leftMotor, &rightMotor, &tapeFollowingSensor)

{
  low_levelAssignMotors(&leftMotor, &rightMotor);
  low_levelAssignLowestLevelObjects(&shiftRegister);
  driveBase.setLineFollowingPID(5.0, 0, 2.0);
  driveBase.followLine(true);
}

void Robot::init() {
  low_level_init();
}

void Robot::update() {
  driveBase.setBaseSpeed(1.5);
  low_level_update();
  driveBase.update();
}


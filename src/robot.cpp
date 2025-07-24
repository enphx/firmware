#include "robot.h"
#include "constants.h"
#include "low_level.h"
#include "low_level/io.h"
#include <Arduino.h>

Robot::Robot()
    : shiftRegister(),

      leftMotor(&shiftRegister, true, PIN_M1_ENC2, PIN_M1_ENC1, PIN_M1_PWM,
                BIT_M1_DIR, 44 * 21, 2, 0.05, 1, 'L'),
      rightMotor(&shiftRegister, false, PIN_M2_ENC2, PIN_M2_ENC1, PIN_M2_PWM,
                 BIT_M2_DIR, 44 * 21, 2, 0.05, 1, 'R'),

      shoulderMotor(&shiftRegister, false, PIN_M3_PWM, ADC_CH_SHOULDER_POT,
                    BIT_M3_DIR, 0.1, 0, 0.0001, 'S'),

      asimuthStepper(&shiftRegister, PIN_TTBL_STEPPER_PULSE, BIT_TTBL_DIR,
                     600 * 16),

      elbowServo(PIN_ELBOW_SERVO_PWM, ELBOW_MIN_PWM, ELBOW_MAX_PWM),

      tapeFollowingSensor(),
      driveBase(&leftMotor, &rightMotor, &tapeFollowingSensor),
      claw(PIN_SHOULDER_DC_PWM, PIN_M3_ENC1),
    arm(&shoulderMotor, &elbowServo, &asimuthStepper)

{
  low_levelAssignMotors(&leftMotor, &rightMotor);
  low_levelAssignLowestLevelObjects(&shiftRegister);
  driveBase.setLineFollowingPID(5.0, 0, 2.0);
  driveBase.followLine(true);
}

void Robot::init() { low_level_init();
  claw.init();
  
}

void Robot::update() {
  driveBase.setBaseSpeed(1.5);
  low_level_update();
  driveBase.update();
}

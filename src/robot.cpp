#include "robot.h"
#include "constants.h"
#include "low_level.h"
#include "low_level/io.h"
#include <Arduino.h>

static const char *TAG = "ROBOT";

#define SPEED_P 2
#define SPEED_I 0
#define SPEED_D 0

Robot::Robot()
    : shiftRegister(),

      leftMotor(&shiftRegister, true, PIN_M1_ENC2, PIN_M1_ENC1, PIN_M1_PWM,
                BIT_M1_DIR, 44 * 21, SPEED_P, SPEED_I, SPEED_D, 'L'),
      rightMotor(&shiftRegister, false, PIN_M2_ENC2, PIN_M2_ENC1, PIN_M2_PWM,
                 BIT_M2_DIR, 44 * 21, SPEED_P, SPEED_I, SPEED_D, 'R'),

      shoulderMotor(&shiftRegister, false, PIN_M3_PWM, ADC_CH_SHOULDER_POT,
                    BIT_M3_DIR, 0.03, 0, 0, 'S'),

      asimuthStepper(&shiftRegister, PIN_TTBL_STEPPER_PULSE, BIT_TTBL_DIR,
                     600 * 8),

      elbowServo(PIN_ELBOW_SERVO_PWM, ELBOW_MIN_PWM, ELBOW_MAX_PWM),

      tapeFollowingSensor(),
      driveBase(&leftMotor, &rightMotor, &tapeFollowingSensor),
      claw(PIN_SHOULDER_DC_PWM, PIN_M3_ENC1),
      arm(&shoulderMotor, &elbowServo, &asimuthStepper)

{
  low_levelAssignMotors(&leftMotor, &rightMotor, &shoulderMotor,
                        &asimuthStepper, &elbowServo);
  low_levelAssignLowestLevelObjects(&shiftRegister);
  driveBase.setLineFollowingPID(0.2, 0, 0);
  driveBase.followLine(true);
}

void Robot::setArmPosition(float m_height, float m_radius, float m_theta,
                           bool relative) {

  if (relative) {
    height += m_height;
    radius += m_radius;
    theta += m_theta;
  } else {
    height = m_height;
    radius = m_radius;
    theta = m_theta;
  }
  arm.setArmPosition(radius, height, theta);
}

void Robot::init() {
  low_level_init();
  claw.init();
}

void Robot::findTape(bool clockwise) {
  updateHighLevel = false;
  
  if (clockwise) {
    leftMotor.setSpeed(-0.5);
    rightMotor.setSpeed(0.5);
  } else {
    leftMotor.setSpeed(0.5);
    rightMotor.setSpeed(-0.5);
  }


}

void Robot::update() {
  if (updateHighLevel == true) {
  driveBase.update();
  }
  low_level_update();
}

int Robot::getTapeFollowingError() { return driveBase.getTapeFollowingError(); }


void Robot::setBaseSpeed(float speed) {
  driveBase.setBaseSpeed(speed);
}

void Robot::setTapeFollowing(bool tapeFollow) {
  driveBase.followLine(tapeFollow);
}

void Robot::setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd) {
  driveBase.setLineFollowingPID(m_Kp, m_Ki, m_Kd);
}

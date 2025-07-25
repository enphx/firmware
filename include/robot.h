#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "low_level/shiftregister.h"
#include "low_level/encodermotor.h"
#include "low_level/tapefollowingsensor.h"
#include "low_level/potentiometermotor.h"
#include "low_level/stepperdriver.h"
#include "driveBase.h"
#include "low_level/io.h"
#include "arm.h"
#include "claw.h"



struct lidarPoint {
  float distance;
  float angle;
};

class Robot {
public:
  Robot();
  void init(void);

  void calibrateStepper() {
    asimuthStepper.calibrate();
  }

  void setStepperSpeed(float speed) {
    asimuthStepper.setSpeed(speed);
  }

  void setArmPosition(float height, float radius, float theta);

  int getTapeFollowingError();

  void setBaseSpeed(float speed);

  void setTapeFollowing(bool tapeFollow);

  void setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd);

  bool stepperIsMoving() {
    return asimuthStepper.moving();
  }

  float getArmTheta() {
    return arm.getTheta();
  }

  int16_t getClawDistance() {
    return claw.getRangeFinderValue();
  }

  void update();

  inline void delay(uint32_t ticks_millis) {
    uint32_t t0 = millis();
    for (uint32_t i = 0; i < ticks_millis; i++) {
      if (millis() - t0 >= ticks_millis) {
        return;
      }
      update();
      vTaskDelay(5);
    }
  }

private:
  EncoderMotor leftMotor, rightMotor;
  PotentiometerMotor shoulderMotor;
  StepperMotor asimuthStepper;
  Servo elbowServo;
  TapeFollowingSensor tapeFollowingSensor;
  DriveBase driveBase;
  ShiftRegister shiftRegister;
  Arm arm;
  Claw claw;

  
};

#endif

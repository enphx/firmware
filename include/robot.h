#ifndef ROBOT_H
#define ROBOT_H

#include "arm.h"
#include "claw.h"
#include "driveBase.h"
#include "low_level/encodermotor.h"
#include "low_level/io.h"
#include "low_level/potentiometermotor.h"
#include "low_level/shiftregister.h"
#include "low_level/stepperdriver.h"
#include "low_level/tapefollowingsensor.h"
#include "constants.h"
#include <Arduino.h>



struct lidarPoint {
  float distance;
  float angle;
};

class Robot {
public:
  Robot();
  void init(void);

  void putArmInDrivePosition(void) {arm.setArmPosition(ARM_DRIVE_RADIUS, ARM_DRIVE_HEIGHT, ARM_DRIVE_THETA);}
  void calibrateStepper() {
    asimuthStepper.calibrate();
  }

  void setStepperSpeed(float speed) {
    asimuthStepper.setSpeed(speed);
  }

  void setArmPosition(float height, float radius, float theta, bool relative);

  int getTapeFollowingError();

  void setBaseSpeed(float speed);

  void scanForPet(float height, float theta1, float theta2);

  inline void grabPet(void) {claw.close();}

  inline void releasePet(void) {claw.open();}

  inline int16_t getDistanceFromObject(void) {return claw.getRangeFinderValue();}

  inline magVector getClawMagnetometerReadings(void) {return claw.getMagnotometerValues();}

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

  float radius, height, theta;
};

#endif

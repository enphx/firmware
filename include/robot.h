#ifndef ROBOT_H
#define ROBOT_H

#include "arm.h"
#include "claw.h"
#include "constants.h"
#include "driveBase.h"
#include "low_level/encodermotor.h"
#include "low_level/io.h"
#include "low_level/potentiometermotor.h"
#include "low_level/shiftregister.h"
#include "low_level/stepperdriver.h"
#include "low_level/tapefollowingsensor.h"
#include "trajectories.h"
#include <Arduino.h>

struct lidarPoint {
  float distance;
  float angle;
};

enum class PIDObject {
  ENCODER_MOTOR,
  DRIVEBASE,
  SHOULDER,
};

class Robot {
public:
  Robot();
  void init(void);

  void calibrateStepper() { asimuthStepper.calibrate(); }

  void setStepperSpeed(float speed) { asimuthStepper.setSpeed(speed); }

  void setArmPosition(float height, float radius, float theta, bool relative);

  void setArmAngles(float asimuthTheta, float shoulderTheta, float elbowTheta);

  void armFollowTrajectory(const Trajectory *trajectory, int numberOfPoints);

  int getTapeFollowingError();

  void setBaseSpeed(float speed);

  void findTape(bool clockwise);

  void scanForPet(float height, float theta1, float theta2);

  inline void grabPet(void) { claw.close(); }

  inline void releasePet(void) { claw.open(); }

  inline magVector getClawMagnetometerReadings(void) {
    return claw.getMagnotometerValues();
  }

  void setTapeFollowing(bool tapeFollow);

  void setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd);

  bool stepperIsMoving() { return asimuthStepper.moving(); }

  float getArmTheta() { return arm.getTheta(); }

  int16_t getClawDistance() { return claw.getRangeFinderValue(); }

  void update();

  void delay(uint32_t ticks_millis);

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

  bool updateHighLevel = true;

  float radius, height, theta;

  void processSerial();

  const static uint MAX_SERIAL_INPUT_SIZE = 256;
  uint8_t serial_message[MAX_SERIAL_INPUT_SIZE];

  bool receive_and_process_serial_messages();
  void send_serial_messages();

  float lastLidarValue = 0;
  uint32_t lastLidarUpdateTime = 0;
};

#endif

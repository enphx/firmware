#ifndef ROBOT_H
#define ROBOT_H

#include "arm.h"
#include "claw.h"
#include "constants.h"
#include "driveBase.h"
#include "include/scanner.h"
#include "low_level/encodermotor.h"
#include "low_level/io.h"
#include "low_level/potentiometermotor.h"
#include "low_level/shiftregister.h"
#include "low_level/stepperdriver.h"
#include "low_level/tapefollowingsensor.h"
#include "robotposition.h"
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

#define SCAN_LEFT true
#define SCAN_RIGHT false

class Robot {
public:
  Robot();
  void init(void);

  void setStepperSpeed(float speed) { asimuthStepper.setSpeed(speed); }

  void setArmPosition(float height, float radius, float theta, bool relative = false);

  void setArmAngles(float asimuthTheta, float shoulderTheta, float elbowTheta);

  void armFollowTrajectory(const Trajectory *trajectory, int numberOfPoints);

  /**
   * This function does not change the theta axis
   */
  void armMoveSmooth(float m_height, float m_radius, int32_t numberOfSteps,
                     int32_t milliseconds, bool relative = false);

  int getTapeFollowingError();

  void setBaseSpeed(float speed);

  void findTape();

  void scanForPet(float height, float theta1, float theta2);

  inline void grabPet(void) { claw.close(); }

  inline void releasePet(void) { claw.open(); }

  inline magVector getClawMagnetometerReadings(void) {
    return claw.getMagnotometerValues();
  }

  inline void resetDistanceTravelled(void) {
    driveBase.resetDistanceTravelled();
  }

  inline float getDistanceTravelled(void) {
    return driveBase.getDistanceTravelled();
  }

  inline RobotPosition getPosition(void) {
    return {driveBase.getPosition(), arm.getPosition()};
  }

  
  inline void setScannerThreshold(uint16_t threshold, bool tiggerAbove) {
    scanner.setThreshold(threshold, tiggerAbove);
  }

  inline bool scannerThresholdTripped(void) { return scanner.thresholdTripper(); }

  void setTapeFollowing(bool tapeFollow);

  void setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd);

  bool stepperIsMoving() { return asimuthStepper.moving(); }

  float getArmTheta() { return arm.getTheta(); }

  int16_t getClawDistance() { return claw.getRangeFinderValue(); }

  inline ScannerPoint getLastScannerPoint() { return lastScannerPoint; }

  inline ScannerPoint getLastLastScannerPoint() { return lastLastScannerPoint; }

  inline void dumpLog(void) {
    scanner.print_log();
  }

  inline void clawHalfOpen() {
    claw.halfOpen();
  }

  inline void startScanning(void) {
    scanning = true;
    scanner.reset();
  }
  inline void stopScanning(void) { scanning = false; }

  inline void flaccid(void) { claw.flaccid(); }

  void update();

  void delay(uint32_t ticks_millis);

  inline void setTurning(bool turning, int direction) {
    driveBase.setTurning(turning, direction);
  }

  inline void moveDistance(float distance) {
    this->resetDistanceTravelled();

    while (this->getDistanceTravelled() < distance) {
      this->delay(1);
    }
  }

  inline void rotateArmRelative(float angle) {
    arm.rotateRelative(angle);
  }

  void turnAngle(float angle);


  // Note: this assumes you are currently looking at a pet.
  inline void locatePet(bool direction, int distance_threshold = 200, float stepperSpeed = 0.4) {

    float dir_mul = -1.0;

    if (direction) {
      dir_mul = 1.0;
    }

    setBaseSpeed(0.0);

    while (stepperIsMoving()) {
      this->delay(1);
    }

    int32_t pet_distance = claw.getRangeFinderValue();

    startScanning();

    setStepperSpeed(stepperSpeed);

    delay(80);
    

    // Start sweeping one way.
    setArmPosition(0, 0, 30.0 * dir_mul, true);


    setScannerThreshold(distance_threshold, false);
    while (!scannerThresholdTripped()) {
      this->delay(1);
    }
 
    setScannerThreshold(distance_threshold, true);
    while (!scannerThresholdTripped()) {
      this->delay(1);
    }
    
    // record value and start sweeping the other way.
    float theta1 = lastScannerPoint.position.armPosition.theta;
    arm.stopStepper();
    setArmPosition(0, 0, -60 * dir_mul, true);

    setScannerThreshold(distance_threshold, false);
    while (!scannerThresholdTripped()) {
      this->delay(1);
    }

    setScannerThreshold(distance_threshold, true);
    while (!scannerThresholdTripped()) {
      this->delay(1);
    }

    // Record pet edge point and stop the stepper.
    float theta2 = lastScannerPoint.position.armPosition.theta;
    arm.stopStepper();

    // Go to arm position with unchanged height and radius,
    // but theta which is between the two points.
    ArmPosition current_pos = arm.getPosition();

    arm.setTheta((theta1 + theta2)/2.0);

    while (stepperIsMoving()) {
      this->delay(1);
    }

    this->delay(80);

    // // convert to inches.
    // setArmPosition(1.5, lastLastScannerPoint.distance * 0.0393701 + 0.3, 0.0, true);
    
    setStepperSpeed(1);
    stopScanning();
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

  bool updateHighLevel = true;

  float radius, height, theta;

  void processSerial();

  const static uint MAX_SERIAL_INPUT_SIZE = 256;
  const static TickType_t xFrequency = pdMS_TO_TICKS(5);
  uint8_t serial_message[MAX_SERIAL_INPUT_SIZE];

  bool receive_and_process_serial_messages();
  void send_serial_messages();

  float lastLidarValue = 0;
  ScannerPoint lastScannerPoint;
  ScannerPoint lastLastScannerPoint;
  uint32_t lastLidarUpdateTime = 0;
  void send_scanner_message();

  Scanner scanner;
  int16_t prev_distance = -1;
  void update_scanner();

  bool scanning = false;
};

#endif

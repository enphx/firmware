#include "robot.h"
#include "constants.h"
#include "low_level.h"
#include "low_level/io.h"
#include <Arduino.h>
#include <cstdio>
#include <cstring>

#include "include/serial/serial_comms.h"

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
      driveBase(&leftMotor, &rightMotor, &tapeFollowingSensor, 0.2, 0, 0),
      claw(PIN_SHOULDER_DC_PWM),
      arm(&shoulderMotor, &elbowServo, &asimuthStepper)

{
  low_levelAssignMotors(&leftMotor, &rightMotor, &shoulderMotor,
                        &asimuthStepper, &elbowServo);
  low_levelAssignLowestLevelObjects(&shiftRegister);
  driveBase.setLineFollowingPID(0.2, 0, 0);
  driveBase.followLine(true);
}

void Robot::armFollowTrajectory(const Trajectory *trajectory, int len) {
  for(int i = 0; i < len; i++){
    const Trajectory& step = trajectory[i];
    arm.setArmAngles(step.asimuthTheta, step.shoulderTheta, step.elbowTheta);
    delay(step.deltaT);
  }
}

void Robot::setPidThing(float m_Kp, float m_Ki, float m_Kd, float m_maxCumulativeError, PIDObject m_pidObject) {
  
    switch (m_pidObject) {
    case PIDObject::SHOULDER:
      shoulderMotor.setPID(m_Kp, m_Ki, m_Kd, m_maxCumulativeError);
    break;
    case PIDObject::DRIVEBASE:
      driveBase.setLineFollowingPID(m_Kp, m_Ki, m_Kd);
    break;
    case PIDObject::ENCODER_MOTOR:
      leftMotor.setPID(m_Kp, m_Ki, m_Kd, m_maxCumulativeError);
      rightMotor.setPID(m_Kp, m_Ki, m_Kd, m_maxCumulativeError);
    break;

  }
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

void Robot::delay(uint32_t milliseconds) {
  int32_t t0 = millis();
  for (int32_t i = 0; i < milliseconds; i++) {
    if (millis() - t0 >= milliseconds) {
      return;
    }
    update();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
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
  driveBase.update();
  low_level_update();

  #ifdef SERIAL_OUTPUT
  processSerial();
  #endif
}

#ifdef SERIAL_OUTPUT

static PIDObject currentTarget = PIDObject::SHOULDER;

void Robot::receive_and_process_serial_messages() {
  
  int len = receive_incoming_message(serial_message, MAX_SERIAL_INPUT_SIZE);

  if (len > 0) {
    write_to_serial(serial_message, len);
    if (serial_message[0] == PID && serial_message[1] == SET) {
      // Update PID message; read floats and whatnot, and then update the according.
      float kp = bits_to_f32(&serial_message[3]);
      float ki = bits_to_f32(&serial_message[7]);
      float kd = bits_to_f32(&serial_message[11]);
      float max_ce = bits_to_f32(&serial_message[15]);

      // char buf[256];
      // int len1 = snprintf(buf, 256, " kp: %f, ki: %f, kd: %f, max_ce: %f ",kp, ki, kd, max_ce);
      // write_to_serial((uint8_t *)buf, len1);

      switch (serial_message[2]) {
        case ENCODER_MOTOR:
          currentTarget = PIDObject::ENCODER_MOTOR;
          leftMotor.setPID(kp, ki, kd, max_ce);
          rightMotor.setPID(kp, ki, kd, max_ce);
          break;
        case DRIVE_BASE:
          currentTarget = PIDObject::DRIVEBASE;
          driveBase.setLineFollowingPID(kp, ki, kd, max_ce);
          break;
        case SHOULDER:
          currentTarget = PIDObject::SHOULDER;
          shoulderMotor.setPID(kp, ki, kd, max_ce);
          break;
      }
    }
  }
}

void copy_4(uint8_t * target, void * source, uint32_t * index) {
  *index = *index + 4;
  memcpy(target, source, 4);
}

void copy_1(uint8_t * target, uint8_t source, uint32_t * index) {
  *index = *index + 1;
  target[0] = source;
}

void Robot::send_serial_messages() {
  float err;
  float setpoint;
  float p_out;
  float i_out;
  float d_out;

  switch (serial_message[2]) {
    case ENCODER_MOTOR:
      leftMotor.getPID(&err, &setpoint, &p_out, &i_out, &d_out);
      break;
    case DRIVE_BASE:
      break;
    case SHOULDER:
      break;
  }
  
  uint8_t message_to_send[256];
  uint32_t i = 0;

  copy_1(message_to_send, MSG_START, &i);
  copy_1(message_to_send, PID, &i);
  copy_4
}

void Robot::processSerial() {
  receive_and_process_serial_messages();
  send_serial_messages();
}

#endif

int Robot::getTapeFollowingError() { return driveBase.getTapeFollowingError(); }

void Robot::setBaseSpeed(float speed) { driveBase.setBaseSpeed(speed); }

void Robot::setTapeFollowing(bool tapeFollow) {
  driveBase.followLine(tapeFollow);
}

void Robot::setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd) {
  driveBase.setLineFollowingPID(m_Kp, m_Ki, m_Kd);
}

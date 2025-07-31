#include "robot.h"
#include "constants.h"
#include "low_level.h"
#include "low_level/io.h"
#include <Arduino.h>

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
      driveBase(&leftMotor, &rightMotor, &tapeFollowingSensor),
      claw(PIN_SHOULDER_DC_PWM),
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



#ifdef SERIAL_OUTPUT

static PIDObject currentTarget = PIDObject::SHOULDER;

bool Robot::receive_and_process_serial_messages() {
  // delay(10);
  
  int len = receive_incoming_message(serial_message, MAX_SERIAL_INPUT_SIZE);

  if (len > 0) {
    if (len == 23 && serial_message[0] == PID && serial_message[1] == SET) {
      // Update PID message; read floats and whatnot, and then update the according.
      // write_to_serial((uint8_t *)"WRITING PID VALS!", 17);

      float set_point = bits_to_f32(&serial_message[3]);
      float kp = bits_to_f32(&serial_message[7]);
      float ki = bits_to_f32(&serial_message[11]);
      float kd = bits_to_f32(&serial_message[15]);
      float max_ce = bits_to_f32(&serial_message[19]);



      // char buf[256];
      // int len1 = snprintf(buf, 256, " kp: %f, ki: %f, kd: %f, max_ce: %f ",kp, ki, kd, max_ce);
      // write_to_serial((uint8_t *)buf, len1);

      switch (serial_message[2]) {
        case ENCODER_MOTOR:
          // write_to_serial((uint8_t*)"ENCODERMOTOR", 11);
          currentTarget = PIDObject::ENCODER_MOTOR;
          leftMotor.setPID(kp, ki, kd, max_ce);
          leftMotor.setSpeed(set_point);
          rightMotor.setPID(kp, ki, kd, max_ce);
          rightMotor.setSpeed(set_point);
          break;
        case DRIVE_BASE:
          // write_to_serial((uint8_t*)"DRVBASE", 7);
          currentTarget = PIDObject::DRIVEBASE;
          driveBase.setLineFollowingPID(kp, ki, kd);
          break;
        case SHOULDER:
          // write_to_serial((uint8_t*)"SHLDR", 5);
          currentTarget = PIDObject::SHOULDER;
          shoulderMotor.setPID(kp, ki, kd);
          break;
      }
    } else if (len == 10 && serial_message[0] == DRIVE_BASE && serial_message[1] == SET) {
      float base_speed = bits_to_f32(&serial_message[2]);
      bool tape_following = (serial_message[6] != 0);

      // if (tape_following) {
      //   write_to_serial((uint8_t*)&"TRUE",4);
      // } else {
      //   write_to_serial((uint8_t*)&"FALSE",5);
      // }

      driveBase.setBaseSpeed(base_speed);
      driveBase.followLine(tape_following);
    } else if (len == 6 && serial_message[0] == TTBL && serial_message[1] == SET) {
      float stepper_angle = bits_to_f32(&serial_message[2]);

      setArmPosition(0, 0, stepper_angle, true);
    }

    // processed a message.
    return true;
  } else {
    // no messages processed.
    return false;
  }
}

inline void copy_4(uint8_t * target, void * source, uint32_t * index) {
  *index = *index + 4;
  memcpy(target, source, 4);
}

inline void copy_1(uint8_t * target, uint8_t source, uint32_t * index) {
  *index = *index + 1;
  target[0] = source;
}

inline void copy_f(uint8_t * target, float f, uint32_t * index) {
  copy_1(target, FLOAT_AHEAD, index);
  copy_4(&target[1], &f, index);
}


void Robot::send_serial_messages() {
  float err;
  float setpoint;
  float p_out;
  float i_out;
  float d_out;

  switch (currentTarget) {
    case PIDObject::ENCODER_MOTOR:
      err = leftMotor.getError();
      setpoint = leftMotor.getSetPoint();
      p_out = leftMotor.getP();
      i_out = leftMotor.getI();
      d_out = leftMotor.getD();
      // leftMotor.getPID(&err, &setpoint, &p_out, &i_out, &d_out);
      break;
    case PIDObject::DRIVEBASE:
      err = driveBase.getError();
      setpoint = driveBase.getSetPoint();
      p_out = driveBase.getP();
      i_out = driveBase.getI();
      d_out = driveBase.getD();
      // driveBase.getPID(&err, &setpoint, &p_out, &i_out, &d_out);
      break;
    case PIDObject::SHOULDER:

      err = shoulderMotor.getError();
      setpoint = shoulderMotor.getSetPoint();
      p_out = shoulderMotor.getP();
      i_out = shoulderMotor.getI();
      d_out = shoulderMotor.getD();
      // shoulderMotor.getPID(&err, &setpoint, &p_out, &i_out, &d_out);
      break;
  }
  
  uint8_t message_to_send[256];
  uint32_t i = 0;

  float odo_x = driveBase.getOdoX();
  float odo_y = driveBase.getOdoY();
  float odo_theta = driveBase.getOdoTheta();
  // driveBase.getOdometry(&odo_x, &odo_y, &odo_theta);

  // write_b_to_serial(MSG_START);
  // write_b_to_serial(PID);
  // write_f_to_serial(err);
  // write_f_to_serial(setpoint);
  // write_f_to_serial(p_out);
  // write_f_to_serial(i_out);
  // write_f_to_serial(d_out);

  // write_b_to_serial(ODOMETRY);
  // write_f_to_serial(odo_x);
  // write_f_to_serial(odo_y);
  // write_f_to_serial(odo_theta);
  // write_b_to_serial(MSG_END);

  // Start message
  copy_1(&message_to_send[i], MSG_START, &i);

  // Send pid values
  copy_1(&message_to_send[i], PID, &i);
  copy_f(&message_to_send[i], err, &i);
  copy_f(&message_to_send[i], setpoint, &i);
  copy_f(&message_to_send[i], p_out, &i);
  copy_f(&message_to_send[i], i_out, &i);
  copy_f(&message_to_send[i], d_out, &i);

  // Send Odometry
  copy_1(&message_to_send[i], ODOMETRY, &i);
  copy_f(&message_to_send[i], odo_x, &i);
  copy_f(&message_to_send[i], odo_y, &i);
  copy_f(&message_to_send[i], odo_theta, &i);

  copy_1(&message_to_send[i], LIDAR, &i);
  // copy_f(&message_to_send[i], (float)claw.getRangeFinderValue(), &i);
  copy_f(&message_to_send[i], lastLidarValue, &i);

  copy_1(&message_to_send[i], MSG_END, &i);

  write_to_serial(message_to_send, i);

}


#endif




void Robot::update() {
  driveBase.update();
  low_level_update();
}

int Robot::getTapeFollowingError() { return driveBase.getTapeFollowingError(); }

void Robot::setBaseSpeed(float speed) { driveBase.setBaseSpeed(speed); }

void Robot::setTapeFollowing(bool tapeFollow) {
  driveBase.followLine(tapeFollow);
}

void Robot::setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd) {
  driveBase.setLineFollowingPID(m_Kp, m_Ki, m_Kd);
}

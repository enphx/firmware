#include "robot.h"
#include "constants.h"
#include "low_level.h"
#include "low_level/io.h"
#include <Arduino.h>

#include "include/serial/serial_comms.h"
#include "low_level/rangefinder.h"

static const char *TAG = "ROBOT";

#define SPEED_P 2.0
#define SPEED_I 0.00002
#define SPEED_D 0.002

Robot::Robot()
    : shiftRegister(),

      leftMotor(&shiftRegister, true, PIN_M1_ENC2, PIN_M1_ENC1, PIN_M1_PWM,
                BIT_M1_DIR, 44 * 21, SPEED_P, SPEED_I, SPEED_D, 'L'),
      rightMotor(&shiftRegister, false, PIN_M2_ENC2, PIN_M2_ENC1, PIN_M2_PWM,
                 BIT_M2_DIR, 44 * 21, SPEED_P, SPEED_I, SPEED_D, 'R'),

      shoulderMotor(&shiftRegister, false, PIN_M3_PWM, ADC_CH_SHOULDER_POT,
                    BIT_M3_DIR, 0.04, 0, 0, 'S'),

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
  driveBase.setLineFollowingPID(0.07, 0, 0.001);
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

void Robot::setArmAngles(float asimuthTheta, float shoulderTheta,
                         float elbowTheta) {
  arm.setArmAngles(asimuthTheta, shoulderTheta, elbowTheta);
}

void Robot::armFollowTrajectory(const Trajectory *trajectory,
                                int numberOfPoints) {
  for (int i = 0; i < numberOfPoints; i++) {
    const Trajectory &point = trajectory[i];
    setArmPosition(point.height, point.radius, point.theta, false);
    this->delay(point.deltaT);
  }
}

void Robot::armMoveSmooth(float m_height, float m_radius, int32_t numberOfSteps,
                   int32_t milliseconds) {
  float deltaHeight = (m_height - height) / numberOfSteps;
  float deltaRadius = (m_radius - radius) / numberOfSteps;
  int32_t deltaT = milliseconds / numberOfSteps;
  for (int i = 1; i <= numberOfSteps; i++) {
    setArmPosition(deltaHeight, deltaRadius, 0, true);
    this->delay(deltaT);
  }
}

void Robot::init() {
  low_level_init();
  claw.init();

#ifdef SERIAL_OUTPUT
  setup_serial_uart();
  vTaskDelay(10);
#endif
}

void Robot::delay(uint32_t milliseconds) {
  int32_t t0 = millis();
  for (int32_t i = 0; i < milliseconds; i++) {
    update();
    vTaskDelay(pdMS_TO_TICKS(5));
    if (millis() - t0 >= milliseconds) {
      return;
    }
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

      float set_point = bits_to_f32(&serial_message[3]);
      float kp = bits_to_f32(&serial_message[7]);
      float ki = bits_to_f32(&serial_message[11]);
      float kd = bits_to_f32(&serial_message[15]);
      float max_ce = bits_to_f32(&serial_message[19]);

      switch (serial_message[2]) {
      case ENCODER_MOTOR:
        currentTarget = PIDObject::ENCODER_MOTOR;
        leftMotor.setPID(kp, ki, kd, max_ce);
        leftMotor.setSpeed(set_point);
        rightMotor.setPID(kp, ki, kd, max_ce);
        rightMotor.setSpeed(set_point);
        break;
      case DRIVE_BASE:
        currentTarget = PIDObject::DRIVEBASE;
        driveBase.setLineFollowingPID(kp, ki, kd);
        break;
      case SHOULDER:
        currentTarget = PIDObject::SHOULDER;
        shoulderMotor.setPID(kp, ki, kd);
        shoulderMotor.setMaxCE(max_ce);
        break;
      }
    } else if (len == 10 && serial_message[0] == DRIVE_BASE &&
               serial_message[1] == SET) {
      float base_speed = bits_to_f32(&serial_message[2]);
      bool tape_following = (serial_message[6] != 0);

      driveBase.setBaseSpeed(base_speed);
      driveBase.followLine(tape_following);
    } else if (len == 6 && serial_message[0] == TTBL &&
               serial_message[1] == SET) {
      float stepper_angle = bits_to_f32(&serial_message[2]);

      setArmPosition(0, 0, stepper_angle, true);
    } else if (len == 10 && serial_message[0] == ARM &&
               serial_message[1] == SET) {
      float r = bits_to_f32(&serial_message[2]);
      float h = bits_to_f32(&serial_message[6]);

      radius = r;
      height = h;
      arm.setArmPosition(radius, height, theta);
    } else if (len == 2 && serial_message[0] == CLAW && serial_message[1] == SET) {
      claw.toggle();
      arm.calibrate();
    }

    return true;
  } else {
    return false;
  }
}

void Robot::send_serial_messages() {
  float err;
  float setpoint;
  float p_out;
  float i_out;
  float d_out;

  switch (currentTarget) {
  case PIDObject::ENCODER_MOTOR:
    err = rightMotor.getError();
    setpoint = rightMotor.getSetPoint();
    p_out = rightMotor.getP();
    i_out = rightMotor.getI();
    d_out = rightMotor.getD();
    break;
  case PIDObject::DRIVEBASE:
    err = driveBase.getError();
    setpoint = driveBase.getSetPoint();
    p_out = driveBase.getP();
    i_out = driveBase.getI();
    d_out = driveBase.getD();
    break;
  case PIDObject::SHOULDER:

    err = shoulderMotor.getError();
    setpoint = shoulderMotor.getSetPoint();
    p_out = shoulderMotor.getP();
    i_out = shoulderMotor.getI();
    d_out = shoulderMotor.getD();
    break;
  }

  uint8_t message_to_send[256];
  uint32_t i = 0;

  float odo_x = driveBase.getOdoX();
  float odo_y = driveBase.getOdoY();
  float odo_theta = driveBase.getOdoTheta();

  copy_1(&message_to_send[i], MSG_START, &i);

  copy_1(&message_to_send[i], PID, &i);
  copy_f(&message_to_send[i], err, &i);
  copy_f(&message_to_send[i], setpoint, &i);
  copy_f(&message_to_send[i], p_out, &i);
  copy_f(&message_to_send[i], i_out, &i);
  copy_f(&message_to_send[i], d_out, &i);

  copy_1(&message_to_send[i], ODOMETRY, &i);
  copy_f(&message_to_send[i], odo_x, &i);
  copy_f(&message_to_send[i], odo_y, &i);
  copy_f(&message_to_send[i], odo_theta, &i);

  // copy_1(&message_to_send[i], LIDAR, &i);

  // copy_f(&message_to_send[i], lastLidarValue, &i);

  copy_1(&message_to_send[i], MSG_END, &i);

  write_to_serial(message_to_send, i);
}

uint32_t last_serial_message_time = 0;

#endif

void Robot::update_scanner() {
  if (!claw.getRangeFinderDataReady()) {
    return;
  }
  int16_t distance = claw.getRangeFinderValue();

  if (distance < 0) {
    distance = MAX_DISTANCE_VALUE;
  }
    ScannerPoint output = scanner.push(distance, getPosition());
    lastLastScannerPoint = lastScannerPoint;
    lastScannerPoint = output;

    ESP_LOGI(TAG, "LIDAR: %i", output.distance);

    send_scanner_message();
}


void Robot::send_scanner_message() {
  #ifdef SERIAL_OUTPUT
  uint8_t message_to_send[256];
  uint32_t i = 0;

  copy_1(&message_to_send[i], MSG_START, &i);
  copy_1(&message_to_send[i], LIDAR, &i);
  
  copy_f(&message_to_send[i], lastScannerPoint.distance, &i);
  copy_f(&message_to_send[i], lastScannerPoint.convolved, &i);

  copy_1(&message_to_send[i], MSG_END, &i);

  write_to_serial(message_to_send, i);
  #endif
}


void Robot::update() {
  driveBase.update();
  low_level_update();
  update_scanner();

#ifdef SERIAL_OUTPUT
  while (receive_and_process_serial_messages())
    ;
  if (millis() - last_serial_message_time >= 50) {
    send_serial_messages();
    last_serial_message_time = millis();
  }
#endif
}

int Robot::getTapeFollowingError() { return driveBase.getTapeFollowingError(); }

void Robot::setBaseSpeed(float speed) { driveBase.setBaseSpeed(speed); }

void Robot::setTapeFollowing(bool tapeFollow) {
  driveBase.followLine(tapeFollow);
}

void Robot::setLineFollowingPID(float m_Kp, float m_Ki, float m_Kd) {
  driveBase.setLineFollowingPID(m_Kp, m_Ki, m_Kd);
}

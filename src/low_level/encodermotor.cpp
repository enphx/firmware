#include "low_level/encodermotor.h"
#include "constants.h"
#include "include/low_level/encodermotor.h"
#include "driver/gpio.h"
#include <Arduino.h>
#include <math.h>

static const char* TAG = "ENCODER MOTOR";

EncoderMotor::EncoderMotor(ShiftRegister* m_shiftReg,
                           const bool m_backwards,
                           const uint8_t m_encoderPin1,
                           const uint8_t m_encoderPin2,
                           const uint8_t m_pwmPin,
                           const uint8_t m_dirBit,
                           const int m_ticksPerRev,
                           const float m_kP,
                           const float m_kI,
                           const float m_kD,
                           const char ID) :
                           shiftReg(m_shiftReg),
                           encoderPin1(m_encoderPin1),
                           encoderPin2(m_encoderPin2),
                           pwmPin(m_pwmPin),
                           dirBit(m_dirBit),
                           ticksPerRev(m_ticksPerRev),
                           kP(m_kP),
                           kI(m_kI),
                           kD(m_kD),
                           ID(ID) {
                             
  if (m_backwards) {
    backwards = -1;
  } else {
    backwards = 1;
  }                             
}

void EncoderMotor::init() {
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  pinMode(pwmPin, OUTPUT);

  ledcAttach(pwmPin, PWM_FREQUENCY, 12);

  attachInterruptArg(encoderPin1, EncoderMotor::genericEncoderHandler, this,
                     CHANGE);
  attachInterruptArg(encoderPin2, EncoderMotor::genericEncoderHandler, this,
                     CHANGE);
  uint8_t s1 = gpio_get_level(gpio_num_t(encoderPin1));
  uint8_t s2 = gpio_get_level(gpio_num_t(encoderPin2));
  previousState = (s1 << 1) | s2; 
}



void IRAM_ATTR EncoderMotor::genericEncoderHandler(void *arg) {
  EncoderMotor *motor = static_cast<EncoderMotor *>(arg);
  if (motor) {
    motor->handleEncoder();
  }
}

void IRAM_ATTR EncoderMotor::handleEncoder() {
  uint8_t s1 = gpio_get_level(gpio_num_t(encoderPin1));
  uint8_t s2 = gpio_get_level(gpio_num_t(encoderPin2));

  uint8_t currentState = (s1 << 1) | s2;
  uint8_t combined = (previousState << 2) | currentState;

  const int8_t lookup[16] = {0,  -1, 1, 0, 1, 0, 0,  -1,
                             -1, 0,  0, 1, 0, 1, -1, 0};

  tickCount += lookup[combined & 0x0F];
  previousState = currentState;
}

// void EncoderMotor::setBackwards(boolean m_backwards) {
//   
// }



int32_t EncoderMotor::update(void) {
  uint64_t t = micros();
  deltaT = (uint32_t)(t - timeLastUpdated);
  currentTickCount = tickCount;
  timeLastUpdated = t;

  int32_t deltaTicks = currentTickCount - previousTickCount;

  currentSpeed = (float)(deltaTicks) / ((float)deltaT) *
                 ticksPerRev * PI * WHEEL_DIAMETER;
  previousTickCount = currentTickCount;

  error = currentSpeed - targetSpeed;
  cumulativeError += error * deltaT;

  float maxCumError = 100 * 0.05;
  cumulativeError = cumulativeError > maxCumError ? maxCumError : cumulativeError;
  cumulativeError = cumulativeError < -maxCumError ? -maxCumError : cumulativeError;

  float power = calculatePID();

  // ESP_LOGI(TAG, "Id: %c, error: %f, cum error: %f, power: %f currentSpeed: %f, kP: %f, kI: %f, kD: %f", ID, error, cumulativeError, power, currentSpeed, kP, kI, kD);

  if (power >= 0) {
    setPWM(power, 1);
  } else {
    setPWM(-power, 0);
  }
  previousError = error;

  return deltaTicks;
}

float EncoderMotor::getCurrentSpeed(void) { return currentSpeed; }

void EncoderMotor::setPID(float m_kP, float m_kI, float m_kD) {
  kP = m_kP;
  kI = m_kI;
  kD = m_kD;
}

uint8_t EncoderMotor::getDirection() { return direction; }

uint8_t EncoderMotor::getDirectionBit() { return dirBit; }

long EncoderMotor::getTickCount() {
  return tickCount;
}

void EncoderMotor::setSpeed(float speed) { targetSpeed = speed * backwards; }

void EncoderMotor::setPWM(float dutyCycle, uint8_t m_direction) {
  if (direction != m_direction) {
    ledcWrite(pwmPin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  direction = m_direction;
  shiftReg->setBit(direction, dirBit);


  dutyCycle = dutyCycle > 1 ? 1 : dutyCycle;

  ledcWrite(pwmPin, (uint32_t)(dutyCycle * 4095));
}

float EncoderMotor::calculatePID(void) {
  if (targetSpeed == 0) {
    return kP * error + kD * (error - previousError) / (deltaT);
  }
  return kP * error + kD * (error - previousError) / (deltaT) +
         kI * cumulativeError;
}

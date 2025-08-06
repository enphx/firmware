#include "low_level/encodermotor.h"
#include "constants.h"
#include "include/low_level/encodermotor.h"
#include "driver/gpio.h"
#include <Arduino.h>
#include <math.h>

static const char* TAG = "ENCODER MOTOR";


static volatile const int8_t lookup[16] = {0,  -1, 1, 0, 1, 0, 0,  -1,
                             -1, 0,  0, 1, 0, 1, -1, 0};

EncoderMotor::EncoderMotor(ShiftRegister* m_shiftReg,
                           const bool m_backwards,
                           const uint8_t m_encoderPin1,
                           const uint8_t m_encoderPin2,
                           const uint8_t m_pwmPin,
                           const uint8_t m_dirBit,
                           const int m_ticksPerRev,
                           const double m_kP,
                           const double m_kI,
                           const double m_kD,
                           const char ID) :
                           shiftReg(m_shiftReg),
                           encoder_data(
                             EncoderData {
                               .pin1 = m_encoderPin1,
                               .pin2 = m_encoderPin2,
                               .prev_state = 0,
                               .tick_count = 0,
                             }),
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
  pinMode(encoder_data.pin1, INPUT);
  pinMode(encoder_data.pin2, INPUT);
  pinMode(pwmPin, OUTPUT);

  ledcAttach(pwmPin, PWM_FREQUENCY, 12);

  attachInterruptArg(encoder_data.pin1, encoderHandler, (void*) &encoder_data,
                     CHANGE);
  attachInterruptArg(encoder_data.pin2, encoderHandler, (void*) &encoder_data,
                     CHANGE);
  uint8_t s1 = gpio_get_level(gpio_num_t(encoder_data.pin1));
  uint8_t s2 = gpio_get_level(gpio_num_t(encoder_data.pin2));
  encoder_data.prev_state = (s1 << 1) | s2; 
}



void IRAM_ATTR encoderHandler(void *arg) {
  volatile EncoderData *data = ((volatile EncoderData *)(arg));

  uint8_t s1 = gpio_get_level(gpio_num_t(data->pin1));
  uint8_t s2 = gpio_get_level(gpio_num_t(data->pin2));

  uint8_t currentState = ((s1 & 1) << 1) | (s2 & 1);
  uint8_t combined = (data->prev_state << 2) | currentState;


  data->tick_count += lookup[combined & 0x0F];
  data->prev_state = currentState;
}

// void EncoderMotor::setBackwards(boolean m_backwards) {
//   
// }



int32_t EncoderMotor::update(void) {

    uint64_t t = micros();
  deltaT = (uint32_t)(t - timeLastUpdated);
  currentTickCount = encoder_data.tick_count;
  timeLastUpdated = t;

  int32_t deltaTicks = currentTickCount - previousTickCount;

  uint32_t timeSinceLastSpeedRead = micros() - lastSpeedReadingTime;
  if (timeSinceLastSpeedRead >= 5000) {

    currentSpeed = (double)(currentTickCount - lastSpeedReadingTicks) /
                   ((double)timeSinceLastSpeedRead) * ticksPerRev * PI *
                   WHEEL_DIAMETER;
    lastSpeedReadingTicks = currentTickCount;
    lastSpeedReadingTime = micros();
    // ESP_LOGI(TAG, "Speed %c: %f", ID, currentSpeed);
  }

  previousTickCount = currentTickCount;

  error = currentSpeed - targetSpeed;
  cumulativeError += error * deltaT;

  cumulativeError = cumulativeError > maxCumError ? maxCumError : cumulativeError;
  cumulativeError = cumulativeError < -maxCumError ? -maxCumError : cumulativeError;

  double power = calculatePID();

  // ESP_LOGI(TAG, "Id: %c, error: %f, cum error: %f, power: %f currentSpeed: %f, kP: %f, kI: %f, kD: %f", ID, error, cumulativeError, power, currentSpeed, kP, kI, kD);

  if (power >= 0) {
    setPWM(power, 1);
  } else {
    setPWM(-power, 0);
  }
  previousError = error;

  return deltaTicks;
}

double EncoderMotor::getCurrentSpeed(void) { return currentSpeed; }

void EncoderMotor::setPID(double m_kP, double m_kI, double m_kD, double max_cum_error) {
  kP = m_kP;
  kI = m_kI;
  kD = m_kD;

  if (max_cum_error >= 0.0) {
    maxCumError = max_cum_error;
  }
}

uint8_t EncoderMotor::getDirection() { return direction; }

uint8_t EncoderMotor::getDirectionBit() { return dirBit; }

long EncoderMotor::getTickCount() {
  return encoder_data.tick_count;
}

void EncoderMotor::setSpeed(double speed) { targetSpeed = speed * backwards; }

void EncoderMotor::setPWM(double dutyCycle, uint8_t m_direction) {
  if (direction != m_direction) {
    ledcWrite(pwmPin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  direction = m_direction;
  shiftReg->setBit(direction, dirBit);


  dutyCycle = dutyCycle > 1 ? 1 : dutyCycle;

  ledcWrite(pwmPin, (uint32_t)(dutyCycle * 4095));
}

double EncoderMotor::calculatePID(void) {
  P = kP * error;
  I = kI * cumulativeError;
  D = kD * (error - previousError)/ (deltaT);

  if (targetSpeed == 0) {
    return P;
  }
  return P + I + D;
}

#include "low_level/stepperdriver.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"
#include <math.h>

StepperMotor::StepperMotor(ShiftRegister *m_shiftRegister, uint8_t m_stepPin,
                           uint8_t m_directionBit,
                           uint32_t m_stepsPerRevolution)
    : shiftregister(m_shiftRegister) {
  stepPin = m_stepPin;
  directionBit = m_directionBit;
  stepsPerRevolution = m_stepsPerRevolution;
}

void StepperMotor::init() {
  pinMode(stepPin, OUTPUT);
  timer = timerBegin(10000);
  timerAttachInterruptArg(timer, &StepperMotor::stepperTimerHandler, this);

  timerAlarm(timer, alarmCallRate, true, 0);
  timerStop(timer);
}

void StepperMotor::setAngle(float angle) {

  if (angle < 0) {
    direction = BACKWARDS;
    angle = angle > -135 ? angle : -135.0f;
    shiftregister->setBit(directionBit, 0);
  } else if (angle > 0) {
    direction = FORWARDS;
    angle = angle < 135 ? angle : 135.0f;
    shiftregister->setBit(directionBit, 1);
  } else {
    return;
  }

  timerStart(timer);

  int32_t targetPosition = angle / 360 * stepsPerRevolution;

  totalSteps = abs(targetPosition - steps);
  stepsRemaining = totalSteps;

  accelStepNum = (maxAngularVelocity * maxAngularVelocity -
                  currentAngularVelocity * currentAngularVelocity) /
                 (2.0f * maxAngularAcceleration);

  deccelStepNum = (maxAngularVelocity * maxAngularVelocity) /
                  (2.0f * maxAngularAcceleration);

  if (accelStepNum + deccelStepNum > totalSteps) {
    accelStepNum = totalSteps / 3;
    deccelStepNum = totalSteps / 3;
  }
}

void StepperMotor::update() {
  if (pauseTimer) {
    timerStop(timer);
    currentAngularVelocity = 0;
    pauseTimer = false;
  }

  if (stepsRemaining > 0) {
    timerAlarm(timer, alarmCallRate, true, 0);
  }
}

void IRAM_ATTR StepperMotor::stepperTimerHandler(void *arg) {
  StepperMotor *stepperMotor = static_cast<StepperMotor *>(arg);
  if (stepperMotor) {
    stepperMotor->handleStep();
  }
}

void IRAM_ATTR StepperMotor::handleStep(void) {

  portENTER_CRITICAL_ISR(&timerMux);

  if (stepsRemaining == 0) {
    pauseTimer = true;
    portEXIT_CRITICAL_ISR(&timerMux);
    return;
  }

  float angularVelocity;

  if (totalSteps - stepsRemaining <= accelStepNum) {

    if (currentAngularVelocity < 0.1) {
      angularVelocity = 0.1;
    } else {

      float deltaAngularVelocity =
          (maxAngularAcceleration * 2.0 * PI) /
          (currentAngularVelocity * stepsPerRevolution);

      angularVelocity = currentAngularVelocity + deltaAngularVelocity;
    }

  } else if (stepsRemaining <= deccelStepNum) {

    float deltaAngularVelocity = -(maxAngularAcceleration * 2.0 * PI) /
                                 (currentAngularVelocity * stepsPerRevolution);

    angularVelocity = currentAngularVelocity + deltaAngularVelocity;

  } else {
    angularVelocity = currentAngularVelocity;
  }

  angularVelocity = maxAngularVelocity > angularVelocity ? angularVelocity
                                                         : maxAngularVelocity;

  currentAngularVelocity = angularVelocity;

  alarmCallRate = 1e6 * (2.0 * PI) / (angularVelocity * stepsPerRevolution);
  stepsRemaining -= 1;
  steps += direction;

  portEXIT_CRITICAL_ISR(&timerMux);

  gpio_set_level((gpio_num_t)stepPin, HIGH);
  ets_delay_us(10);
  gpio_set_level((gpio_num_t)stepPin, LOW);
}

bool StepperMotor::moving() { return stepsRemaining > 0; }

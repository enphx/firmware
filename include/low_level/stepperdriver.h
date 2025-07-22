#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H
#include "low_level/shiftregister.h"
#include <Arduino.h>

enum DIRECTION { FORWARDS = 1, BACKWARDS = -1 };

class StepperMotor {
public:
  StepperMotor(ShiftRegister *m_shiftRegister, uint8_t m_stepPin,
               uint8_t m_directionBit, uint32_t m_stepsPerRevolution);

  void calibrate(void);

  void update(void);

  void init(void);

  void setAngle(float angle);

  bool moving(void);

private:

  void  handleStep(void);

  static void  stepperTimerHandler(void *arg);

  hw_timer_t *timer = nullptr;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

  ShiftRegister* const shiftregister;
  int8_t direction;
  uint32_t stepsPerRevolution;
  volatile int32_t steps = 0;
  volatile int32_t stepsRemaining;
  volatile bool pauseTimer = false;
  int32_t totalSteps;
  uint8_t stepPin, directionBit;
  float maxAngularVelocity = 5, maxAngularAcceleration = 1000;
  float currentAngularVelocity;

  uint32_t accelStepNum, deccelStepNum;

  uint32_t alarmCallRate = 1000;
};

#endif

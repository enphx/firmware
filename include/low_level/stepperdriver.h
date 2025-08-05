#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H
#include "driver/gptimer.h"
#include "low_level/shiftregister.h"
#include <Arduino.h>

enum DIRECTION { FORWARDS = 1, BACKWARDS = -1 };

class StepperMotor {
public:
  StepperMotor(ShiftRegister *m_shiftRegister, uint8_t m_stepPin,
               uint8_t m_directionBit, uint32_t m_stepsPerRevolution);

  void init(void);

  void setAngle(float angle);

  void setSpeed(float speed);

  float getAngle();

  bool moving(void);

private:
  void handleStepHigh(void);

  void handleStepLow(void);

  gptimer_handle_t gptimer = NULL;
  gptimer_config_t timer_config;
  gptimer_alarm_config_t alarm_config;

  // volatile bool timerIsRunning = false;
  volatile bool stepState = false;

  float targetAngle = 0.0;
  float speed = 0.0;

  ShiftRegister *const shiftregister;

  int8_t direction;
  uint32_t stepsPerRevolution;

  uint8_t directionBit;

  const uint8_t min_steps = 14;

};

#endif

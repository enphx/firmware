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

  void calibrate(void);

  void init(void);

  void setAngle(float angle);

  void setSpeed(float speed);

  float getAngleRelative();

  float getAngleAbsolute();

  bool moving(void);

private:
  void handleStepHigh(void);

  void handleStepLow(void);

  static bool stepperTimerHandler(gptimer_handle_t timer,
                                  const gptimer_alarm_event_data_t *edata,
                                  void *user_ctx);

  gptimer_handle_t gptimer = NULL;
  gptimer_config_t timer_config;
  gptimer_alarm_config_t alarm_config;

  volatile bool timerIsRunning = false;
  volatile bool stepState;

  ShiftRegister *const shiftregister;
  int8_t direction;
  uint32_t stepsPerRevolution;
  volatile int32_t steps = 0;
  volatile int32_t stepsRemaining = 0;
  int32_t totalSteps;
  uint8_t stepPin, directionBit;
  float angularVelocity;
  uint32_t alarmCallPeriod = 0;
};

#endif

#include "low_level/stepperdriver.h"
#include "constants.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp32/rom/ets_sys.h"
#include "low_level/core0.h"
#include "low_level/io.h"

static const char *TAG = "STEPPER";

StepperMotor::StepperMotor(ShiftRegister *m_shiftRegister, uint8_t m_stepPin,
                           uint8_t m_directionBit,
                           uint32_t m_stepsPerRevolution)
    : shiftregister(m_shiftRegister) {
  stepPin = m_stepPin;
  directionBit = m_directionBit;
  stepsPerRevolution = m_stepsPerRevolution;
  
  timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 1000 * 1000,
  };


alarm_config = {
    .alarm_count = (uint64_t)(1e6 / (float)(stepsPerRevolution) /
                          TURNTABLE_ROTATIONS_PER_SECOND * 0.5),
    .flags = {
      .auto_reload_on_alarm = true,
    },
  };


  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
}

void StepperMotor::setSpeed(float speed) {

  if (timerIsRunning) {
    ESP_ERROR_CHECK(gptimer_stop(gptimer));

    if (stepState == true) {
      delayMicroseconds(8);
      this->handleStepLow();
    }
    timerIsRunning = false;
  }

  alarm_config.alarm_count = (uint64_t)(1e6 / (float)(stepsPerRevolution) /
                          TURNTABLE_ROTATIONS_PER_SECOND * 0.5 / speed);

  gptimer_set_alarm_action(gptimer, &alarm_config);
}

void StepperMotor::init() {
  pinMode(stepPin, OUTPUT);
  ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = stepperTimerHandler,
  };

  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, this));
  ESP_ERROR_CHECK(gptimer_enable(gptimer));
  ESP_ERROR_CHECK(gptimer_start(gptimer));
  timerIsRunning = true;

  stepState = false;
}

void StepperMotor::setAngle(float angle) {

  if (timerIsRunning) {
    ESP_ERROR_CHECK(gptimer_stop(gptimer));

    if (stepState == true) {
      delayMicroseconds(8);
      this->handleStepLow();
    }
    timerIsRunning = false;
  }

  calibrate();

  if (angle < getAngleRelative()) {
    direction = BACKWARDS;
    angle = angle > -TTBL_MAX_ANGLE ? angle : -TTBL_MAX_ANGLE;
    shiftregister->setBit(0, directionBit);
  } else if (angle > 0) {
    direction = FORWARDS;
    angle = angle < TTBL_MAX_ANGLE ? angle : TTBL_MAX_ANGLE;
    shiftregister->setBit(1, directionBit);
  } else {
    totalSteps = 0;
    stepsRemaining = 0;
    return;
  }

  int32_t targetPosition = angle / 360 * stepsPerRevolution;


  totalSteps = abs(targetPosition - steps);
  stepsRemaining = totalSteps;

  if (stepsRemaining == 0) {
    return;
  }

  ESP_LOGI(TAG, "angle: %f, steps: %i, targetPosition: %i, totalSteps: %i, stepsRemaining: %i", angle, steps, targetPosition, totalSteps, stepsRemaining);

  if (!timerIsRunning) {
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    timerIsRunning = true;
  }
}

bool IRAM_ATTR StepperMotor::stepperTimerHandler(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
    void *user_ctx) {

  StepperMotor *stepperMotor = static_cast<StepperMotor *>(user_ctx);

  if (!stepperMotor) {
    return false;
  }

  if (!stepperMotor->stepState) {
    stepperMotor->handleStepHigh();
    return false;
  }
  stepperMotor->handleStepLow();
  return false;
}

void IRAM_ATTR StepperMotor::handleStepHigh(void) {

  gpio_set_level((gpio_num_t)stepPin, HIGH);
  stepState = true;
}

void IRAM_ATTR StepperMotor::handleStepLow(void) {
  gpio_set_level((gpio_num_t)stepPin, LOW);
  stepState = false;

  stepsRemaining -= 1;
  steps += direction;

  if (stepsRemaining <= 0) {
    gptimer_stop(gptimer);
    timerIsRunning = false;
  }
}

bool StepperMotor::moving() { return stepsRemaining > 0; }

float StepperMotor::getAngleRelative() {
  return steps * 360.0 / stepsPerRevolution;
}

float StepperMotor::getAngleAbsolute() {
  return (float)(get_convolved_value(ADC_CH_TURNTABLE_POT) - 1855) * DEGREES_PER_POT_TICK_TTBL;
  // return (float)get_convolved_value(ADC_CH_TURNTABLE_POT);
}

void StepperMotor::calibrate() {
  steps = getAngleAbsolute() / 360.0 * stepsPerRevolution;
  ESP_LOGI(TAG, "steps: %i", steps);
}

#include "low_level/stepperdriver.h"
#include "constants.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"

static const char *TAG = "STEPPER";

StepperMotor::StepperMotor(ShiftRegister *m_shiftRegister, uint8_t m_stepPin,
                           uint8_t m_directionBit,
                           uint32_t m_stepsPerRevolution)
    : shiftregister(m_shiftRegister) {
  stepPin = m_stepPin;
  directionBit = m_directionBit;
  stepsPerRevolution = m_stepsPerRevolution;
  alarmCallPeriod = (int)(1e6 / (float)(stepsPerRevolution) /
                          TURNTABLE_ROTATIONS_PER_SECOND * 0.5);

  timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 1000 * 1000,
  };

  alarm_config = {
      .alarm_count = alarmCallPeriod,
      .flags =
          {
              .auto_reload_on_alarm = true,
          },
  };

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
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

  if (angle < steps * 360.0 / stepsPerRevolution) {
    direction = BACKWARDS;
    angle = angle > -135 ? angle : -135.0f;
    shiftregister->setBit(0, directionBit);
  } else if (angle > 0) {
    direction = FORWARDS;
    angle = angle < 135 ? angle : 135.0f;
    shiftregister->setBit(1, directionBit);
  } else {
    totalSteps = 0;
    stepsRemaining = 0;
    return;
  }

  int32_t targetPosition = angle / 360 * stepsPerRevolution;

  totalSteps = abs(targetPosition - steps);
  stepsRemaining = totalSteps;

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

  if (stepsRemaining == 0) {
    gptimer_stop(gptimer);
    timerIsRunning = false;
  }
}

bool StepperMotor::moving() { return stepsRemaining > 0; }

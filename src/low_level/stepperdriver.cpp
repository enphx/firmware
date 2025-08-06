#include "low_level/stepperdriver.h"
#include "constants.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "low_level/core0.h"
#include "low_level/io.h"

static const char *TAG = "STEPPER";

static volatile uint32_t steps_to_take = 0;
static volatile bool step_pulse_phase = false;
static volatile uint8_t step_pin;

StepperMotor::StepperMotor(ShiftRegister *m_shiftRegister, uint8_t m_stepPin,
                           uint8_t m_directionBit,
                           uint32_t m_stepsPerRevolution)
    : shiftregister(m_shiftRegister) {
  step_pin = m_stepPin;
  directionBit = m_directionBit;
  stepsPerRevolution = m_stepsPerRevolution;
  
  timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 100 * 1000,
  };


alarm_config = {
    .alarm_count = (uint64_t)(1e5 / (double)(stepsPerRevolution) /
                          TURNTABLE_ROTATIONS_PER_SECOND * 0.5),
    .flags = {
      .auto_reload_on_alarm = true,
    },
  };


  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
}




static bool IRAM_ATTR stepperTimerHandler(gptimer_handle_t timer,
                                   const gptimer_alarm_event_data_t *edata,
                                   void *user_ctx) {

  if (steps_to_take == 0) {
    return false;
  }

  if (step_pulse_phase == false) {
    gpio_set_level((volatile gpio_num_t)step_pin, HIGH);
    step_pulse_phase = true;
  } else {
    gpio_set_level((volatile gpio_num_t)step_pin, LOW);
    step_pulse_phase = false;
    steps_to_take = steps_to_take - 1;
  }

  return false;
}

void StepperMotor::setSpeed(double speed) {

  this->speed = speed;

  alarm_config.alarm_count = (uint64_t)(1e5 / (double)(stepsPerRevolution) /
                          TURNTABLE_ROTATIONS_PER_SECOND / speed * 0.5);

  gptimer_set_alarm_action(gptimer, &alarm_config);

  // ESP_LOGI(TAG, "Set stepper speed to %f", speed);
}

void StepperMotor::init() {

  // ESP_LOGI(TAG, "Stepper motor init. Step pin: %u", step_pin);
  pinMode(step_pin, OUTPUT);
  ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = stepperTimerHandler,
  };

  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
  ESP_ERROR_CHECK(gptimer_enable(gptimer));
  ESP_ERROR_CHECK(gptimer_start(gptimer));

}

void StepperMotor::stop() {
  steps_to_take = 0;
}

void StepperMotor::setAngle(double angle) {

  // ESP_LOGI(TAG, "Setting stepper motor angle to %f", angle);

  targetAngle = angle;

  int32_t target_steps = (angle - getAngle()) / 360.0 * stepsPerRevolution;

  // Bang the bits to set the direction of the stepper.
  if (target_steps > 0) {
    shiftregister->setBit(1, directionBit);
  } else {
    shiftregister->setBit(0, directionBit);
  }

  target_steps = target_steps < 0 ? -target_steps : target_steps;


  // ESP_LOGI(TAG, "Target steps: %i", target_steps);

 

 // Only move if we are being told to move more than the minimum
  // (this is to avoid annoying oscillations due to backlash).
  if (target_steps <= min_steps) {
    return;
  }

  
  // ESP_LOGI(TAG, "Set steps_to_take.");
  // This should make the steps start stepping.
  steps_to_take = target_steps;
}

bool StepperMotor::moving() { return steps_to_take > 0; }

double StepperMotor::getAngle() {
  // double angle = (double)((int32_t)(get_convolved_value(ADC_CH_TURNTABLE_POT)) - 1855) * DEGREES_PER_POT_TICK_TTBL;  
  // ESP_LOGI(TAG, "Stepper angle: %f", angle);
  return (double)((int32_t)(get_convolved_value(ADC_CH_TURNTABLE_POT)) - 1855) * DEGREES_PER_POT_TICK_TTBL;
}

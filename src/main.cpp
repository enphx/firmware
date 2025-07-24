#include "claw.h"
#include "low_level/core0.h"
#include "low_level/io.h"
#include "low_level/potentiometermotor.h"
#include "low_level/servodriver.h"
#include "robot.h"
#include <Arduino.h>

static const char *TAG = "main";

ShiftRegister shiftRegister;

PotentiometerMotor motor(&shiftRegister, false, 3, 4, 2, 0.1, 0, 0.0001, 'S');
void statusTask(void *arg) {
  for (;;) {
    gpio_set_level((gpio_num_t)4, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level((gpio_num_t)4, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  xTaskCreate(statusTask, "statusLed task", 4096, NULL, 3, NULL);
  shiftRegister.init();
  core0_init();
  motor.init();

  delay(3000);
}

#define DELAY_TIME 2000

void loop() {
  motor.setAngle(45);

  while (abs(motor.getAngle() - 45) > 1) {
    motor.update();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  for (int i = 0; i < 1000; i++) {
    motor.update();
    vTaskDelay(1);
  }
  motor.setAngle(90);

  while (abs(motor.getAngle() - 90) > 1) {
    motor.update();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  for (int i = 0; i < 1000; i++) {
    motor.update();
    vTaskDelay(1);
  }
}

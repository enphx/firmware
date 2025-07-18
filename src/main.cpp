#include <Arduino.h>
#include "include/low_level/encodermotor.h"
#include "include/low_level.h"

EncoderMotor motor1();

uint32_t prev_t = 0;
uint32_t t = 0;
float speed = 1.4;

void setup() {
  delay(100);
  low_level_init();
  t = micros();
  set_speed(speed);
}

#define DELAY_TIME 2000


void loop() {
  t = millis();
  if (t - prev_t > 1000) {
    prev_t = t;
    speed = -speed;
    set_speed(speed);
  }
  low_level_update();
  vTaskDelay(5);
}

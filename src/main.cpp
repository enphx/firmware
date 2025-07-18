#include <Arduino.h>
#include "include/low_level/encodermotor.h"
#include "include/low_level.h"

EncoderMotor motor1();

uint32_t prev_t = 0;
uint32_t t = 0;
float speed = 0.1;

void setup() {
  delay(100);
  low_level_init();
  t = micros();
  set_speed(speed);
}

#define DELAY_TIME 2000


void loop() {
//  t = millis();
//  if (t - prev_t > 100) {
//    prev_t = t;
//    if (speed >= 2) {
//      speed = 0.1;
//    }
//
//    if (speed < 0.001) {
//      speed -= 0.1;
//    } else {
//      speed += 0.1;
//    }
//    set_speed(speed);
//  }
  low_level_update();
  vTaskDelay(5);
}

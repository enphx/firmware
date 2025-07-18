#include <Arduino.h>
#include "include/low_level.h"
#include "robot.h"

uint32_t prev_t = 0;
uint32_t t = 0;
float speed = 1.4;

Robot robot;

void setup() {
  delay(100);
  robot.init();
}

#define DELAY_TIME 2000


void loop() {
  robot.update();
  vTaskDelay(5);
}

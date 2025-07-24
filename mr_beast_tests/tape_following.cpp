#include "driveBase.h"
#include "low_level/core0.h"
#include "low_level/io.h"
#include "robot.h"
#include <Arduino.h>

static const char *TAG = "main";

Robot robot;

void statusTask(void *arg) {
  for (;;) {
    gpio_set_level((gpio_num_t)4, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level((gpio_num_t)4, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


#define TICK_AVG_COUNT 10

void robotTask(void *arg) {
  robot.setArmPosition(15, 8, 0);

  robot.delay(500);

  robot.setBaseSpeed(2);
  robot.setTapeFollowing(true);

  uint32_t ticks = 0;
  uint64_t t0 = micros();

  while (1) {

    if (ticks >= TICK_AVG_COUNT) {
      uint64_t t = micros();
      // ESP_LOGI(TAG, "loop time, avgd. %u vals, (μs): %f", TICK_AVG_COUNT, (float)(t - t0)/TICK_AVG_COUNT);
      t0 = micros();
      ticks = 0;
    }

    ticks++;
    
    robot.update();
    vTaskDelay(5);
  }
}

void activationEventTask(void *arg) {
  delay(2000);
  xTaskCreatePinnedToCore(robotTask, "robot task", 8192, NULL, 1, NULL, 1);
  vTaskDelete(NULL);
}

void setup() {
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(37, INPUT_PULLDOWN);
  digitalWrite(4, LOW);
  robot.init();
  ESP_LOGI(TAG, "robot initialized");
  xTaskCreatePinnedToCore(statusTask, "statusLed task", 4096, NULL, 3, NULL, 1);
  delay(100);
  xTaskCreatePinnedToCore(activationEventTask, "activationTask", 4096, NULL, 3,
                          NULL, 1);
}

void loop() {}

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

void robotTask(void *arg) {
  ESP_LOGI(TAG, "started robot task");
  robot.setArmPosition(15, 8, 0);

  for(int i = 0; i < 1000; i++) {
    robot.update();
    vTaskDelay(1);
  }
  vTaskDelay(1);
  for (;;) {
    for (int i = 0; i < 70; i++) {
      robot.setArmPosition(15 - 0.1 * i, 8, 0);
      robot.update();
      vTaskDelay(1);
    }
    for (int i = 0; i < 70; i++) {
      robot.setArmPosition(8 + 0.1 * i, 8, 0);
      robot.update();
      vTaskDelay(1);
    }
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

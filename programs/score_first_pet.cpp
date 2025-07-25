#include "include/petdetect.h"
#include "low_level/core0.h"
#include "low_level/io.h"
#include "robot.h"
#include <Arduino.h>

// NOTE: pet detection was being run with a kernel of [-1, 0, 1].

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

#define SCAN_HEIGHT 3.7
#define SCAN_RADIUS 5.5

void robotTask(void *arg) {

  // for (;;) {
  //   robot.setArmPosition(15, 8, 0,false);
  //   robot.delay(1000);
  // }

  ESP_LOGI(TAG, "started robot task");
  robot.setArmPosition(4, 11, 0, false);
  robot.delay(5000);
  robot.setTapeFollowing(true);
  robot.setBaseSpeed(1);
  int32_t startTime = millis();

  while (millis() - startTime <= 10000) {
    robot.setArmPosition(4, 11, 0, false);
    robot.delay(100);
  }

  robot.setBaseSpeed(0);

  robot.setArmPosition(6, 0, 0, true);
  robot.delay(1000);

  robot.setArmPosition(10, 11, -90, false);
  robot.delay(1500);
  robot.setArmPosition(SCAN_HEIGHT, SCAN_RADIUS, -90, false);

  robot.delay(2000);

  robot.setBaseSpeed(0.5);
  robot.delay(200);

  int disctance = robot.getClawDistance();

  while (disctance > 200 || disctance < 5) {
    disctance = robot.getClawDistance();
    robot.update();
    vTaskDelay(1);
  }

  robot.setBaseSpeed(0);
  robot.delay(1000);
  robot.setArmPosition(SCAN_HEIGHT, SCAN_RADIUS, -60, false);

  robot.delay(2000);

  robot.setStepperSpeed(0.3);
  robot.setArmPosition(SCAN_HEIGHT, SCAN_RADIUS, -110, false);

  int16_t prev_val = 0;

  PetDetectOutput output1, output2, prevOutput = {0, 0, 0, 0};

  int edge_threshold = 500;
  int distance_threshold = 400;

  int timer0 = millis();

  while (millis() - timer0 < 4000) {
    robot.update();
    int16_t val = robot.getClawDistance();
    val = val < 1 ? 3000 : val;
    if (val != prev_val) {
      PetDetectOutput output = pet_detect_push(robot.getArmTheta(), val);
      if (output.convolved >= edge_threshold &&
          prevOutput.distance <= distance_threshold) {
        output1 = prevOutput;
      } else if (output.convolved <= -edge_threshold &&
                 output.distance <= distance_threshold) {
        output2 = output;
      }
      prev_val = val;
      prevOutput = output;
    }
    vTaskDelay(5);
  }

  // for (;;) {
  //   pet_detect_print();
  //
  //   ESP_LOGI(TAG, "DETECTED PET EDGE1: angle1: %f, distance1: %i, conv1: %i",
  //            output1.angle, output1.distance, output1.convolved);
  //   ESP_LOGI(TAG, "DETECTED PET EDGE2: angle2: %f, distance2: %i, conv2: %i",
  //            output2.angle, output2.distance, output2.convolved);
  // }

  robot.setStepperSpeed(1);

  robot.setArmPosition(SCAN_HEIGHT, SCAN_RADIUS,
                       (output1.angle + output2.angle) / 2, false);
  robot.delay(2000);
  float distance = (float)(robot.getClawDistance()) * 0.0393701 * 1.8;
  robot.setArmPosition(2.5, 0, 0, true);
  robot.delay(2000);
  robot.setArmPosition(0, distance - 3, 0, true);
  robot.delay(400);
  robot.grabPet();
  robot.delay(1000);
  robot.setArmPosition(15, 8, 0, false);
  robot.delay(1000);
  robot.setBaseSpeed(1);
  robot.delay(8000);
  robot.setBaseSpeed(0);
  robot.setArmPosition(-2, 2, 90, true);
  robot.delay(1500);
  robot.releasePet();

  for (;;) {
    robot.delay(1000);
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

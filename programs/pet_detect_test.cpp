#include "low_level/core0.h"
#include "low_level/io.h"
#include "include/petdetect.h"
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

#define SCAN_HEIGHT 7
#define SCAN_RADIUS 7
#define TEST_ANGLE 90

void robotTask(void *arg) {

 
  ESP_LOGI(TAG, "started robot task");
  while (1) {
     ESP_LOGI(TAG, "moving to 90 deg");
     robot.setArmPosition(12, 10, TEST_ANGLE);
     robot.delay(2000);
     ESP_LOGI(TAG, "moving to 90 deg 2");
     robot.setArmPosition(12, 10, TEST_ANGLE);
     robot.delay(2000);
     robot.setArmPosition(SCAN_HEIGHT + 2, SCAN_RADIUS, TEST_ANGLE);
     robot.delay(2000);
     robot.setArmPosition(SCAN_HEIGHT, SCAN_RADIUS, 110);

     robot.delay(2000);

     robot.setStepperSpeed(0.3);
     robot.setArmPosition(SCAN_HEIGHT, SCAN_RADIUS, 50);

     int16_t prev_val = 0;

     PetDetectOutput output1, output2 = {0, 0, 0, 0};

     int threshold = 140;

     while (robot.stepperIsMoving()) {
       robot.update();
       int16_t val = robot.getClawDistance();
       if (val != prev_val) {
         PetDetectOutput output = pet_detect_push(robot.getArmTheta(), val);
         if (output.convolved >= 140) {
           output1 = output;
         } else if (output.convolved <= -140) {
           output2 = output;
         }
         prev_val = val;
       }
       vTaskDelay(5);
     }

     ESP_LOGI(TAG, "angle1: %f, angle2: %f", output1.angle, output2.angle);

     pet_detect_print();
     pet_detect_reset();
     // robot.delay(2000);
     robot.setArmPosition(SCAN_HEIGHT + 3, SCAN_RADIUS, (output1.angle + output2.angle)/2 + 5);
     robot.delay(5000);
     // pet_detect_push(robot.getArmTheta(), robot.getClawDistance());
     // vTaskDelay(5);
    
    ESP_LOGI(TAG, "%f, %i", robot.getArmTheta(), robot.getClawDistance());
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

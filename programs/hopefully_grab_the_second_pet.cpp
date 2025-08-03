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

  for (;;) {
    robot.delay(1);
  }

  // robot.releasePet();
  // robot.delay(400);
  // robot.flaccid();
  // robot.setArmPosition(6.5, 3.5, 60, false);
  // robot.delay(3000);
  // robot.setTapeFollowing(true);
  // robot.setBaseSpeed(1.5);
  // robot.startScanning();
  // robot.delay(50);
  //

  ESP_LOGI(TAG, "started robot task");
  robot.calibrateArm();
  robot.setArmPosition(3.5, 12, 0, false);
  robot.delay(5000);
  robot.setTapeFollowing(true);
  robot.releasePet();

  robot.setArmPosition(3.5, 12, 0, false);
  robot.setBaseSpeed(1.5);
  robot.resetDistanceTravelled();

  while (robot.getDistanceTravelled() <= 1.6) {
    robot.delay(1);
  }

  robot.setBaseSpeed(0.8);

  robot.armFollowTrajectory(scanOutisdeDoor, scanOutisdeDoorLength);
  robot.startScanning();
  while (robot.getLastScannerPoint().distance > 200) {
    robot.delay(1);
  }
  robot.stopScanning();
  robot.resetDistanceTravelled();
  while (robot.getDistanceTravelled() < 0.02) {
    robot.delay(1);
  }
  robot.setBaseSpeed(0);
  robot.armMoveSmooth(3.5 + 2, 5.5 + 5.5, 10, 800);
  robot.grabPet();
  robot.delay(1000);
  robot.resetDistanceTravelled();
  robot.setBaseSpeed(1.5);
  robot.armFollowTrajectory(pickupPetOne, pickupPetOneLength);

  while (robot.getDistanceTravelled() < 0.8) {
    robot.delay(1);
  }
  robot.armFollowTrajectory(scorePetOne, scorePetOneLength);
  robot.releasePet();
  robot.delay(200);
  robot.setArmPosition(6.6, 3.5, 60, false);
  robot.flaccid();
  while (robot.getDistanceTravelled() < 1.5) {
    robot.delay(1);
  }

  robot.delay(300);
  robot.startScanning();
  robot.delay(100);

  robot.setScannerThreshold(300, false);
  while (!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }
  robot.setScannerThreshold(300, true);

  while (!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }
  robot.setBaseSpeed(0);

  robot.armFollowTrajectory(scanPetTwo, scanPetTwoLength);
  robot.setBaseSpeed(0.8);
  robot.setScannerThreshold(180, false);
  while(!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }

  robot.stopScanning();
  robot.resetDistanceTravelled();
  while (robot.getDistanceTravelled() < 0.02) {
    robot.delay(1);
  }

  robot.armMoveSmooth(3.5 + 2, 5.5 + robot.getClawDistance() / 25.4, 10, 300);
  robot.grabPet();

  for(;;) {
    robot.delay(1);
  }
  while (robot.getLastScannerPoint().distance > 250) {
    robot.delay(1);
  }

  robot.setBaseSpeed(0.8);
  robot.delay(50);

  while (robot.getLastScannerPoint().distance < 200) {
    robot.delay(1);
  }
  robot.setBaseSpeed(0.4);
  robot.resetDistanceTravelled();
  while(robot.getDistanceTravelled() < 0.05) {
    robot.delay(1);
  }
  robot.setBaseSpeed(0);
  robot.setArmPosition(0, 9, 0, true, false);
  robot.delay(500);
  robot.grabPet();
  for (;;) {
    robot.delay(5000);
  }

}

void activationEventTask(void *arg) {
  delay(2000);
  xTaskCreatePinnedToCore(robotTask, "robot task", 8192, NULL, 1, NULL, 1);
  vTaskDelete(NULL);
}

void setup() {
  pinMode(4, OUTPUT);
  robot.init();
  ESP_LOGI(TAG, "robot initialized");
  xTaskCreatePinnedToCore(statusTask, "statusLed task", 4096, NULL, 3, NULL, 1);
  delay(100);
  xTaskCreatePinnedToCore(activationEventTask, "activationTask", 4096, NULL, 3,
                          NULL, 1);
}

void loop() {}

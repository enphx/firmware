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

  ESP_LOGI(TAG, "started robot task");


  // Setup initial conditions.
  robot.setArmPosition(7.1, 14.5, 0, false);
  robot.setBaseSpeed(0);
  robot.setTapeFollowing(true);
  robot.releasePet();
  robot.delay(400);
  robot.flaccid();
  robot.startScanning();

  // Wait for start signal from switch.
  while (get_convolved_value(ADC_CH_IR_BEACON_R) < 1000) {
    robot.delay(1);
  }

  robot.stopScanning();

  // Start line following forwards, travelling for some distance until we hit the door.
  robot.setArmPosition(7.1, 14.5, 0, false);
  robot.setBaseSpeed(1.5);
  robot.resetDistanceTravelled();
  while (robot.getDistanceTravelled() <= 1.5) {
    robot.delay(1);
  }

  // We are now through the door
  robot.releasePet();
  robot.setBaseSpeed(0.5);
  robot.armFollowTrajectory(scanOutisdeDoor, scanOutisdeDoorLength);
  robot.startScanning();
  robot.delay(100);

  // Scan for first pet
  robot.setScannerThreshold(200, false);
  while (robot.scannerThresholdTripped() == false) {
    robot.delay(1);
  }

  // Stop and grab first pet when it is detected.
  robot.stopScanning();
  robot.resetDistanceTravelled();
  robot.setBaseSpeed(0);
  robot.armMoveSmooth(3.5 + 1, 8 + 5, 10, 400);
  robot.grabPet();
  robot.delay(700);

  // Raise pet up, and move up the ramp.
  robot.resetDistanceTravelled();
  robot.setBaseSpeed(1.5);
  robot.armFollowTrajectory(pickupPetOne, pickupPetOneLength);
  while (robot.getDistanceTravelled() < 0.8) {
    robot.delay(1);
  }

  // Score the pet as we move up the ramp.
  robot.armFollowTrajectory(scorePetOne, scorePetOneLength);
  robot.releasePet();

  // Set arm in position to scan for second pet.
  robot.delay(200);
  robot.setArmPosition(8, 3, 70, false);
  robot.flaccid();
  while (robot.getDistanceTravelled() < 1.5) {
    robot.delay(1);
  }
  robot.delay(300);
  robot.startScanning();
  robot.delay(100);

  // Scan for window wall.
  robot.setScannerThreshold(200, false);
  while (!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }

  // Slow down and scan for window hole.
  robot.setBaseSpeed(0.5);
  robot.setScannerThreshold(300, true);

  while (!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }

  robot.delay(80);

  
  // Scan for window far edge.
  robot.resetDistanceTravelled();
  robot.setScannerThreshold(300, false);
  while (!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }

  robot.delay(80);

  // Scan for window ending
  robot.setScannerThreshold(400, true);
  while (!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }
  
  robot.stopScanning();

  

  // Stop, pick up pet 2, and score it through the window.

  robot.setBaseSpeed(0);

  robot.setArmPosition(0, 0, 12, true);

    // Grab the pet
  robot.armMoveSmooth(8 - 4, 3 + 6.5, 10, 300);
  robot.grabPet();
  robot.delay(800);

    // Pull the pet back.
  robot.armMoveSmooth(6 + 3, 4, 10, 200);
    // Rotate to window.
  robot.setArmPosition(0, 0, 20, true);
    // Put thru window and drop.
  robot.armMoveSmooth(9 +1.3, 9 + 4, 50, 1000);
  robot.releasePet();
  robot.delay(300);
    // Pull pet back
  robot.armMoveSmooth(9, 9, 50, 500);
  robot.delay(1000);


  // Scan for the third pet.
  robot.setArmPosition(8, 3, 60, false);
  robot.delay(200);
  robot.startScanning();
  robot.delay(800);
  robot.setBaseSpeed(1.5);
  robot.delay(200);
  robot.setScannerThreshold(300, false);
  while (robot.scannerThresholdTripped() == false) {
    robot.delay(1);
  }
  robot.setBaseSpeed(0);

  // Grab pet 3 and put it in the basket.
  robot.armMoveSmooth(8 + 4, 3 + 9, 10, 500);
  robot.grabPet();
  robot.delay(300);
  robot.armFollowTrajectory(storePetThree, storePetThreeLength);
  robot.releasePet();

  for (;;) {
    robot.delay(1);
  }
}

void setup() {
  pinMode(4, OUTPUT);

  robot.init();
  ESP_LOGI(TAG, "robot initialized");
  xTaskCreatePinnedToCore(statusTask, "statusLed task", 4096, NULL, 3, NULL, 1);
  delay(100);
  xTaskCreatePinnedToCore(robotTask, "robot task", 8192, NULL, 1, NULL, 1);
}

void loop() {}

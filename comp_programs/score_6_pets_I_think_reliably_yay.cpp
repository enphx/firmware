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

  float dist = 0;



  // Setup initial conditions.
  robot.setArmPosition(7.1, 14.5, 0, false);
  robot.setBaseSpeed(0);
  robot.setTapeFollowing(true);
  robot.releasePet();
  robot.delay(400);
  robot.flaccid();
  robot.startScanning();


  while (get_convolved_value(ADC_CH_IR_BEACON_R) < 1000) {
    robot.delay(2);
  }


  robot.stopScanning();

  // Start line following forwards, travelling for some distance until we hit the door.
  robot.setArmPosition(7.1, 14.5, 0, false);
  robot.setBaseSpeed(1.5);
  robot.resetDistanceTravelled();
  while (robot.getDistanceTravelled() <= 1.58) {
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

  // Locate and grab it.
  robot.locatePet(SCAN_LEFT);
  // robot.setArmPosition(1.5, robot.getLastScannerPoint().distance * 0.0393701 + 0.3, 0, true);
  robot.armMoveSmooth(1.5, robot.getLastScannerPoint().distance * 0.0393701 + 0.5, 40, 400, true);
  robot.grabPet();
  robot.delay(500);

 

  // // Stop and grab first pet when it is detected.
  // robot.stopScanning();
  // robot.resetDistanceTravelled();
  // robot.setBaseSpeed(0);
  // robot.armMoveSmooth(3.5 + 1, 8 + 5, 10, 400);
  // robot.grabPet();
  // robot.delay(700);


  // Raise pet up, and move up the ramp.
  robot.resetDistanceTravelled();
  robot.setBaseSpeed(1.5);
  robot.armFollowTrajectory(pickupPetOne, pickupPetOneLength);
  while (robot.getDistanceTravelled() < 0.8) {
    robot.delay(1);
  }

  // Score the pet as we move up the ramp.
  robot.armFollowTrajectory(scorePetOne, scorePetOneLength);
  robot.delay(350);
  robot.releasePet();

  // Set arm in position to scan for second pet.
  robot.delay(650);
  robot.setArmPosition(8, 3, 60, false);
  robot.flaccid();
  while (robot.getDistanceTravelled() < 1.4) {
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



  // Turn off the tape
  float petTwoAngle = 10;
  robot.setBaseSpeed(1);
  robot.setTapeFollowing(false);
  robot.turnAngle(-petTwoAngle);


  // Travel some distance
  robot.resetDistanceTravelled();
  while (robot.getDistanceTravelled() < 0.12){
    robot.delay(1);
  }

  robot.setBaseSpeed(0.0);

  robot.armMoveSmooth(7, 6.5, 30, 300);
  robot.armMoveSmooth(3.5, 6.5, 10, 200);

  robot.resetDistanceTravelled();
  robot.setBaseSpeed(0.4);
  robot.setScannerThreshold(150, false);

  robot.delay(80);


  // Look for pet 2 while not going too far (cliff ahead and not tape following)
  while (!robot.scannerThresholdTripped() && robot.getDistanceTravelled() < 0.2) {
    robot.delay(1);
  }

  robot.setBaseSpeed(0.0);



  // Scan and grab pet 2.
  robot.startScanning();

  robot.locatePet(SCAN_RIGHT, 150);


  robot.armMoveSmooth(1.5, robot.getLastScannerPoint().distance * 0.0393701 + 0.15, 40, 500, true);

  robot.grabPet();
  robot.delay(500);
  robot.armMoveSmooth(15, 8, 30, 200);
  robot.setArmPosition(15, 8, 0);

  robot.setBaseSpeed(-1);
  robot.resetDistanceTravelled();
  robot.delay(10);
  while (robot.getDistanceTravelled() > -0.12) {
    robot.delay(1);
  }


  
  robot.setBaseSpeed(1);
  robot.setTapeFollowing(false);
  robot.turnAngle(petTwoAngle);
  robot.setBaseSpeed(0.0);

  robot.setBaseSpeed(-1.0);
  robot.resetDistanceTravelled();
  robot.delay(10);
  while (robot.getDistanceTravelled() > -0.5) {
    robot.delay(1);
  }

  
  robot.setBaseSpeed(0.0);


  // Drop pet 2, find tape, and keep going!

  robot.setArmPosition(15, 8, 90);
  robot.delay(1200);
  robot.armMoveSmooth(9, 12.5, 30, 300);
  robot.delay(200);
  robot.releasePet();
  robot.delay(600);
  robot.flaccid();
  robot.setArmPosition(9, 9, 30);
  robot.setBaseSpeed(1);
  robot.turnAngle(10);
  robot.findTape();
  robot.setBaseSpeed(0);

  
  robot.setTapeFollowing(true);
  robot.setArmPosition(9, 11, 0);
  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }
  robot.delay(80);
  robot.armMoveSmooth(8.3, 6.5, 30, 300);

  robot.setBaseSpeed(1);

  robot.resetDistanceTravelled();
  robot.delay(80);
  while (robot.getDistanceTravelled() < 0.75){
    robot.delay(1);
  }

  robot.setArmPosition(8.3, 5.6, 45);

  robot.setBaseSpeed(0.4);

  // Detect Pet 3.
  robot.startScanning();
  robot.delay(100);
  robot.setScannerThreshold(200, false);
  while (!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }

  

  // Scan and pick up pet 3.
  robot.locatePet(SCAN_RIGHT, 230);

  dist = robot.getLastScannerPoint().distance * 0.0393701 + 0.3;

  robot.armMoveSmooth(11, 6.5 + dist, 40, 500);

  robot.grabPet();
  robot.delay(600);
  

  robot.armMoveSmooth(15, 6.5 + dist, 30, 300);
  robot.armMoveSmooth(15, 6, 30, 600);

  robot.setArmPosition(10, 6, -10);
  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }
  robot.delay(100);

  robot.armMoveSmooth(9.2, 4.5, 30, 300);

  robot.delay(400);
  robot.releasePet();
  robot.delay(600);
  robot.flaccid();

  robot.armMoveSmooth(13, 5.5, 30, 400);



  // Detect pet 4
  robot.setArmPosition(15, 6.3, 90);
  robot.delay(200);
  robot.setArmPosition(15, 6.5, 90);

  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }

  robot.setBaseSpeed(1.0);
  robot.resetDistanceTravelled();
  delay(10);
  while (robot.getDistanceTravelled() < 0.4) {
    robot.delay(1);
  }
  robot.setBaseSpeed(0.0);
  robot.armMoveSmooth(3.5, 6.5, 30, 300);
  robot.setBaseSpeed(0.4);


  robot.startScanning();
  delay(50);
  robot.resetDistanceTravelled();
 
  robot.setScannerThreshold(200, false);
  while (!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }


  // Scan and pick up pet 4.
  
  robot.locatePet(SCAN_RIGHT, 250, 0.14);

  robot.delay(10);
  robot.grabPet();
  robot.delay(300);
  robot.clawHalfOpen();
  robot.armMoveSmooth(9, 8, 30, 300);
  robot.flaccid();
  robot.rotateArmRelative(-2.8);
  
  robot.startScanning();
  delay(400);

  // robot.armMoveSmooth(4, robot.getLastScannerPoint().distance * 0.0393701 + 0.5, 40, 650, true);
  robot.armMoveSmooth(10, 8 + robot.getLastScannerPoint().distance * 0.0393701 + 0.5, 40, 650);


  // robot.releasePet();
  robot.delay(20);
  robot.grabPet();

  robot.delay(300);


  // Lift pet and put in basket.
  robot.armMoveSmooth(12, 5, 30, 600);


  robot.setArmPosition(12, 6, 5);

  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }

  robot.delay(100);

  robot.armMoveSmooth(9.2, 4.5, 30, 300);

  robot.delay(300);

  robot.releasePet();
  robot.delay(300);
  robot.flaccid();


  // move out of the way
  robot.armMoveSmooth(12, 6.4, 10, 200);


  // scan for pet 5
  robot.setArmPosition(12, 6.4, 80);

  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }

  robot.setBaseSpeed(0.4);

  
  robot.armMoveSmooth(4.6, 6.3, 30, 600);

  robot.setScannerThreshold(200, false);
  while (!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }


  // Stop and scan for pet 5
  robot.setBaseSpeed(0);

  robot.startScanning();
  robot.delay(100);

  robot.locatePet(SCAN_RIGHT);

  robot.startScanning();
  robot.delay(100);
 


  // Fucking slime pet 5
  dist = robot.getLastLastScannerPoint().distance * 0.0393701 + 0.3;

  robot.armMoveSmooth(4.5, 6.5, 20, 200);
  robot.armMoveSmooth(4.5, 6.5 + dist, 30, 400);

  robot.delay(300);

  robot.grabPet();

  robot.delay(400);

  robot.armMoveSmooth(10.8, 5.5, 30, 400);


  // Drop pet 5
  robot.setArmPosition(10.8, 5.5, 10);

  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }

  robot.delay(300);
  robot.releasePet();
  robot.delay(400);

  robot.setArmPosition(13, 5.6, 0);

  robot.delay(200);


  robot.setArmPosition(15, 8, -70);

  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }

  robot.setBaseSpeed(0.4);

  robot.armMoveSmooth(4.6, 6.3, 30, 500);

  robot.startScanning();
  robot.delay(100);

  robot.setScannerThreshold(200, false);
  while (!robot.scannerThresholdTripped()) {
    robot.delay(1);
  }


  // Stop and scan for pet 6
  robot.setBaseSpeed(0);

  robot.locatePet(SCAN_LEFT);

  robot.startScanning();
  robot.delay(100);

  
  // Fucking slime pet 6.
  dist = robot.getLastLastScannerPoint().distance * 0.0393701 + 0.3;

  robot.armMoveSmooth(4.5, 6.5, 20, 300);
  robot.armMoveSmooth(4.5, 6.5 + dist, 30, 600);

  robot.delay(300);

  robot.grabPet();

  robot.delay(400);

  robot.armMoveSmooth(15, 8, 30, 400);


  robot.setArmPosition(15, 8, 0.0);


  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }

  robot.setBaseSpeed(-0.3);

  robot.resetDistanceTravelled();

  robot.delay(100);
  while (robot.getDistanceTravelled() > -0.08) {
    robot.delay(1);
  }

  robot.setBaseSpeed(0.3);
  robot.turnAngle(13);
  robot.setBaseSpeed(-0.3);

  robot.resetDistanceTravelled();

  while (robot.getDistanceTravelled() > 0.08) {
    robot.delay(1);
  }

  robot.setBaseSpeed(0.8);
  robot.turnAngle(200);

  robot.findTape();

  robot.setBaseSpeed(0.0);

  robot.delay(2000);
  // robot.setTapeSide(tapeSide::Left);;

  
  // Follow tape home
  robot.setTapeFollowing(true);
  robot.setBaseSpeed(1);

  robot.waitForAngle(50);

  robot.resetDistanceTravelled();

  while(robot.getDistanceTravelled() < 1.15) {
    robot.delay(1);
  }

  robot.setArmPosition(13, 12, -90);

  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }

  robot.delay(200);

  robot.releasePet();

  robot.delay(300);

  robot.setArmPosition(15, 8, -4);

  robot.waitForAngle(-60);

  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }

  robot.setBaseSpeed(0);
  robot.armMoveSmooth(7.1, 14.5, 40, 500);

  robot.setBaseSpeed(1);

  robot.waitForAngle(-60);

  robot.setArmPosition(7.1, 14.5, -4);

  while (robot.stepperIsMoving()) {
    robot.delay(1);
  }


  robot.resetDistanceTravelled();

  while (robot.getDistanceTravelled() < 1.4) {
    robot.delay(1);
  }

  robot.setBaseSpeed(0);

 



  while (1) {
    robot.setBaseSpeed(0.0);
    robot.delay(100);
  }







  

 
  
}

void setup() {
  pinMode(4, OUTPUT);

  robot.init();
  ESP_LOGI(TAG, "robot initialized");
  xTaskCreatePinnedToCore(statusTask, "statusLed task", 2048, NULL, 3, NULL, 1);
  delay(100);
  xTaskCreatePinnedToCore(robotTask, "robot task", 4 * 8192, NULL, 1, NULL, 1);
}

void loop() {}

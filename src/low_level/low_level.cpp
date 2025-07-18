#include "core0.h"
#include "esp_log.h"
#include "include/low_level.h"
#include "include/low_level/io.h"
#include "include/low_level/encodermotor.h"
#include "include/low_level/shiftregister.h"
#include <Arduino.h>

static const char* TAG = "LOW LEVEL";

ShiftRegister shiftReg;

#define MOTOR_TICKS_PER_REV

EncoderMotor motorL(
  &shiftReg,
  1, //not backwards
  PIN_M1_ENC1,
  PIN_M1_ENC2,
  PIN_M1_PWM,
  BIT_M1_DIR,
  44 * 21, // ticks per revolution; we kinda made this up lol...
  2.1,  // kP
  0.01, // kI
  0.02,  // kD
  'L'
);

EncoderMotor motorR(
  &shiftReg,
  0, //not backwards
  PIN_M2_ENC1,
  PIN_M2_ENC2,
  PIN_M2_PWM,
  BIT_M2_DIR,
  44 * 21, // ticks per revolution; we kinda made this up lol...
  2.1,  // kP
  0.01, // kI
  0.02,  // kD
  'R'
);

void low_level_init() {
  ESP_LOGI(TAG, "init...");
  core0_init();
  motorL.init();
  motorR.init();
}

void set_speed(float speed) {
  motorL.setSpeed(speed);
  motorR.setSpeed(speed);
}

void print_adc_vals() {
  ESP_LOGI(TAG, "0: %hu, 1: %hu, 2: %hu, 3: %hu, 4: %hu, 5: %hu, 6: %hu, 7: %hu,",
           get_convolved_value(0),
           get_convolved_value(1),
           get_convolved_value(2),
           get_convolved_value(3),
           get_convolved_value(4),
           get_convolved_value(5),
           get_convolved_value(6),
           get_convolved_value(7)
         );
}

void low_level_update(){
  motorR.update();
  motorL.update();
}

#include "include/low_level/shiftregister.h"
#include "driver/gpio.h"
#include "include/low_level/io.h"

static const char* TAG = "SHIFT_REGISTER";

void ShiftRegister::init() {
  gpio_set_direction((gpio_num_t)PIN_SHIFT_REG_DATA, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)PIN_SHIFT_REG_CLK, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)PIN_SHIFT_REG_STRB, GPIO_MODE_OUTPUT);
  delay(10);
  writeCurrentValue();
}

bool ShiftRegister::testForChanges(uint16_t bits, uint16_t bitmask) {
  return (bits & bitmask) != (currentValue & bitmask);
}

void print_16_bit_number(uint16_t num) {
  char bin_str[17];
  for (int i = 0; i < 16; i++) {
    bin_str[15 - i] = ((num >> i) & 0b1) ? '1' : '0';
  }
  bin_str[16] = '\0';
  ESP_LOGI(TAG, "0b%s", bin_str);
}

void ShiftRegister::setBit(bool bitValue, uint8_t bit) {
  if (bit < 16) {
    // ESP_LOGI(TAG, "currentValue is 0x%x. Writing bit %u at position %u.", currentValue, bitValue, bit);
    uint16_t bitmask = ((0b1) << bit);
    uint16_t shifted_bit_value = ((bitValue & 0b1) << bit);
    if (testForChanges(shifted_bit_value, bitmask)) {
      // ESP_LOGI(TAG, "shifted val: 0x%x, inv. shifted val: 0x%x", shifted_bit_value, ~shifted_bit_value);
      // ESP_LOGI(TAG, "actaully wrote value!");
      currentValue &= ~bitmask;
      currentValue |= shifted_bit_value;
      // ESP_LOGI(TAG, "current value is now 0x%x", currentValue),
      writeCurrentValue();
    }
  }
}

// HAS NOT BEEN TESTED! (but should work lol?)
void ShiftRegister::setBits(uint16_t bits, uint16_t bitmask) {
  if (testForChanges(bits, bitmask)) {
    currentValue &= ~bitmask;
    currentValue |= (bits & bitmask);
    writeCurrentValue();
  }
}

void ShiftRegister::writeCurrentValue() {
  // TIME TO BANG THE BITS!!!
  print_16_bit_number(currentValue);
  gpio_set_level((gpio_num_t )PIN_SHIFT_REG_STRB, 0);
  gpio_set_level((gpio_num_t)PIN_SHIFT_REG_CLK, 0);
  gpio_set_level((gpio_num_t)PIN_SHIFT_REG_CLK, 1);
  gpio_set_level((gpio_num_t)PIN_SHIFT_REG_CLK, 0);
  for (int i = 15; i >=0; i--) {
    gpio_set_level((gpio_num_t)PIN_SHIFT_REG_DATA, (currentValue >> i) & 0b00000001);
    gpio_set_level((gpio_num_t)PIN_SHIFT_REG_CLK, 1);
    gpio_set_level((gpio_num_t)PIN_SHIFT_REG_CLK, 0);
  }
  gpio_set_level((gpio_num_t)PIN_SHIFT_REG_STRB, 1);
  gpio_set_level((gpio_num_t)PIN_SHIFT_REG_STRB, 0);
}

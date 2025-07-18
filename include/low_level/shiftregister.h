#ifndef SHIFT_REGISTER_H
#define SHIFT_REGISTER_H

#include <Arduino.h>

class ShiftRegister {
public:

  // Bit should be from 0 to 15 inclusive.
  // Does nothing on invalid bit.
  void setBit(bool bitValue, uint8_t bit);
  void setBits(uint16_t bits, uint16_t bitmask);

private:
  void writeCurrentValue();
  uint16_t currentValue = 0;
};

#endif

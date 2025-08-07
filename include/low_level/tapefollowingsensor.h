#ifndef REFLECTANCE_SENSOR_H
#define REFLECTANCE_SENSOR_H
#include "low_level/core0/adc.h"
#include <Arduino.h>

#define LEFT_REFLECTANCE_SENSOR_CHANNEL 3
#define RIGHT_REFLECTANCE_SENSOR_CHANNEL 0
#define UPPER_REFLECTANCE_READING_THRESHOLD 2930

enum class tapeState {
  Difference,
  Inversion,
  OutOfBounds,
};

enum class tapeSide {
  Left,
  Right,
};

class TapeFollowingSensor {
public:
  TapeFollowingSensor(void);

  int getError(void);

  void update(void);

  const tapeState getTapeState(void);

  const tapeSide getSide(void);

  inline void setTapeFollowingSide(tapeSide side) {
    this->Side = side;
  }
  
private:
  int error;
  uint16_t lastDifferenceReadingLeft, lastDifferenceReadingRight;
  tapeState Distance;
  tapeSide Side;
};

#endif

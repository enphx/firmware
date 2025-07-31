#include "low_level/tapefollowingsensor.h"
#include "low_level/core0.h"

static const char *TAG = "TAPEFOLLOWING";

TapeFollowingSensor::TapeFollowingSensor(void) {}

int TapeFollowingSensor::getError(void) { return error; }

void TapeFollowingSensor::update(void) {

  uint16_t reflectanceReadingLeft =
      get_convolved_value(LEFT_REFLECTANCE_SENSOR_CHANNEL);
  uint16_t reflectanceReadingRight =
      get_convolved_value(RIGHT_REFLECTANCE_SENSOR_CHANNEL);

  int leftError = reflectanceReadingLeft;
  int rightError = reflectanceReadingRight;

  // ESP_LOGI(TAG, "LEFT %u, \t RIGHT %u", reflectanceReadingLeft, reflectanceReadingRight);

  if (reflectanceReadingLeft > UPPER_REFLECTANCE_READING_THRESHOLD &&
      reflectanceReadingRight > UPPER_REFLECTANCE_READING_THRESHOLD) {

    Distance = tapeState::OutOfBounds;
  } else if (reflectanceReadingLeft > UPPER_REFLECTANCE_READING_THRESHOLD) {

    rightError = 3 * lastDifferenceReadingRight - 2 * reflectanceReadingLeft;
    Distance = tapeState::Inversion;
    Side = tapeState::Left;
  } else if (reflectanceReadingRight > UPPER_REFLECTANCE_READING_THRESHOLD) {

    leftError = 3 * lastDifferenceReadingLeft - 2 * reflectanceReadingLeft;
    Distance = tapeState::Inversion;
    Side = tapeState::Right;
  } else {
    Distance = tapeState::Difference;
    Side = reflectanceReadingLeft > reflectanceReadingRight ? tapeState::Left
                                                            : tapeState::Right;
    lastDifferenceReadingLeft = reflectanceReadingLeft;
    lastDifferenceReadingRight = reflectanceReadingRight;
  }

  error = leftError - rightError;
}

const tapeState TapeFollowingSensor::getTapeState(void) { return Distance; }

const tapeState TapeFollowingSensor::getSide(void) { return Side; }

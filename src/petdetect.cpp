
#include "include/petdetect.h"
#include "esp_log.h"
#include "include/convfilter.h"


static const char* TAG = "PET DETECT";

#define BUF_SIZE 1000

static float angles[BUF_SIZE];
static int16_t distances[BUF_SIZE];
static int32_t convolved[BUF_SIZE];

static bool first_value = true;

static uint32_t ind = 0;


#define KERN_SIZE 2
int32_t kern1[KERN_SIZE] = {
  -1,
  1,
};

static ConvFilter<int32_t, KERN_SIZE> conv_filter(kern1);


PetDetectOutput get_val(uint32_t i) {
  if (i < BUF_SIZE) {
    return {
      i,
      angles[i],
      distances[i],
      convolved[i],
    };
  }

  return {0, 0, 0, 0};
}


void pet_detect_reset() {
  first_value = true;
  for (int i = 0; i < BUF_SIZE; i++) {
    angles[i] = 0;
    distances[i] = 0;
    convolved[i] = 0;
  }
  ind = 0;
}

PetDetectOutput pet_detect_push(float angle, int16_t distance) {
  if (first_value) {
    for (int i = 0; i < KERN_SIZE; i++) {
      conv_filter.eval(distance);
    }
    first_value = false;
  }
  convolved[ind] = conv_filter.eval(distance);
  angles[ind] = angle;
  distances[ind] = distance;
  ind = (ind + 1) >= BUF_SIZE ? BUF_SIZE - 1 : ind + 1;

  return {
    ind - 1,
    angle,
    distance,
    convolved[ind - 1],
  };
}

void pet_detect_print() {
  ESP_LOGI(TAG, "angle, distance, convolved");
  for (int i = 0; i < BUF_SIZE; i++) {
    if (angles[i] == 0 && distances[i] == 0 && convolved[i] == 0) {
      return;
    }
    ESP_LOGI(TAG, "%f, %i, %i", angles[i], distances[i], convolved[i]);
  }
}

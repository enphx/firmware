
#include <Arduino.h>


struct PetDetectOutput {
  uint32_t index;
  float angle;
  int16_t distance;
  int32_t convolved;  
};


void pet_detect_reset();
PetDetectOutput pet_detect_push(float angle, int16_t distance);
void pet_detect_print();

#ifndef ARM_H
#define ARM_H
#include <Arduino.h>

// TODO: Finish arm header file
class Arm {
public:
  Arm(void);

  void begin();

  void update(void);

  void setArmPosition(float radius, float height, float theta);

private:

  float targetRadius, targetHeight, targetTheta;
  float currentRadius, currentHeight, currentTheta;
};

#endif

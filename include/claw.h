#ifndef CLAW_H
#define CLAW_H

#include <Arduino.h>

struct MagnetometerReadings {
  float x;
  float y;
  float z;
};

class Claw {
public:
  Claw(void);
  
  void begin();
  
  void open(void);
  
  void close(void);

  float getRangeFindetValue(void);

  MagnetometerReadings getMagnotometerValues(void);
  
};

#endif

#ifndef SCANNER_H
#define SCANNER_H

#include <Arduino.h>
#include <cstring>
#include "convfilter.h"
#include "include/robotposition.h"

struct ScannerPoint {
  int16_t distance;
  int32_t convolved;  
  RobotPosition position;
};



#define KERN_SIZE 2
static int32_t kern1[KERN_SIZE] = {
  -1,
  1,
};


#define BUF_SIZE 1000

class Scanner {
public:
  inline ScannerPoint getVal(uint32_t index) {
    return index < BUF_SIZE ? buffer[index] : ScannerPoint {-1,-1,{{-1,-1,-1},{-1,-1,-1}}};
  }

  inline void reset() {
    first_value = true;
    memset(buffer, 0, sizeof(buffer));
    buf_index = 0;
  }

  ScannerPoint push(int16_t distance, RobotPosition robot_position);

  void print_log();

private:
  ScannerPoint buffer[BUF_SIZE];
  uint32_t buf_index = 0;
  bool first_value = true;

  ConvFilter<int32_t, KERN_SIZE> conv_filter = ConvFilter<int32_t, KERN_SIZE>(kern1);

 
};

#endif

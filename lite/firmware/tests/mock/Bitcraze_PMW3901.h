// Minimal Bitcraze PMW3901 declarations used by companion.ino -- syntax check only.
#pragma once
#include <cstdint>
class Bitcraze_PMW3901 {
 public:
  explicit Bitcraze_PMW3901(uint8_t cspin);
  bool begin();
  void readMotionCount(int16_t* deltaX, int16_t* deltaY);
};

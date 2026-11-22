// Minimal Pololu VL53L1X declarations used by companion.ino -- syntax check only.
#pragma once
#include <cstdint>
class VL53L1X {
 public:
  enum DistanceMode { Short, Medium, Long, Unknown };
  enum RangeStatus : uint8_t { RangeValid = 0 };
  struct RangingData { uint16_t range_mm; RangeStatus range_status; float peak_signal_count_rate_MCPS; float ambient_count_rate_MCPS; };
  RangingData ranging_data;
  void setTimeout(uint16_t);
  bool init(bool io_2v8 = true);
  bool setDistanceMode(DistanceMode);
  bool setMeasurementTimingBudget(uint32_t);
  void startContinuous(uint32_t);
  bool dataReady();
  uint16_t read(bool blocking = true);
};

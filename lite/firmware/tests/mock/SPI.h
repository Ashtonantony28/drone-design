// Minimal SPI declarations used by companion.ino -- syntax check only.
#pragma once
#include <cstdint>
#define SPI_MODE3 3
struct SPISettings { SPISettings(uint32_t, uint8_t, uint8_t) {} };
struct SPIClass {
  void begin();
  void beginTransaction(SPISettings);
  void endTransaction();
  uint8_t transfer(uint8_t);
};
extern SPIClass SPI;

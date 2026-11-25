// Minimal WiFiUDP declarations used by companion.ino -- syntax check only.
#pragma once
#include <cstddef>
#include <cstdint>
#include "WiFi.h"
struct WiFiUDP {
  uint8_t begin(uint16_t);
  int parsePacket();
  int read(uint8_t*, size_t);
  IPAddress remoteIP();
  uint16_t remotePort();
  int beginPacket(IPAddress, uint16_t);
  size_t write(const uint8_t*, size_t);
  int endPacket();
};

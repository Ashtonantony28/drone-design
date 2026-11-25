// Minimal arduino-esp32 WiFi declarations used by companion.ino -- syntax check only.
#pragma once
#include <cstdint>
struct String { const char* c_str() const; };
struct IPAddress { String toString() const; };
struct WiFiClass {
  bool softAP(const char*, const char*);
  IPAddress softAPIP();
  int softAPgetStationNum();
  bool setSleep(bool);
};
extern WiFiClass WiFi;

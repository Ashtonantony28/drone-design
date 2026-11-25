// Minimal Bluepad32 (Arduino) declarations used by lite_fc.ino -- syntax check only.
// Method names follow Bluepad32 4.x Controller; confirm against the installed version.
#pragma once
#include <cstdint>
struct Controller {
  bool isConnected() const;
  bool hasData() const;
  void disconnect();
  int32_t axisX() const;
  int32_t axisY() const;
  int32_t axisRX() const;
  int32_t axisRY() const;
  int32_t throttle() const;
  bool a() const;
  bool b() const;
  bool x() const;
  bool y() const;
  uint16_t miscButtons() const;
  void playDualRumble(uint16_t delayedStartMs, uint16_t durationMs, uint8_t weakMagnitude, uint8_t strongMagnitude);
};
typedef Controller* ControllerPtr;
typedef void (*GTKCallback)(ControllerPtr);
struct Bluepad32 {
  void setup(GTKCallback onConnect, GTKCallback onDisconnect);
  bool update();
  void enableVirtualDevice(bool);
};
extern Bluepad32 BP32;

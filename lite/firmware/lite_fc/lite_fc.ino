// NF-3D Lite flight controller: ESP32 (D1 mini) + MPU-6050 + Xbox controller over Bluetooth.
//
// Board: "ESP32 Dev Module" from the Bluepad32 board package (see lite/firmware/README.md).
// Libraries: nf_core and lite_fc from lite/firmware/libraries (copy them into Arduino/libraries).
//
// Everything that decides how the drone flies is in lite_fc/FlightCore, which is unit-tested
// and closed-loop tested on a PC (lite/firmware/tests). This file only moves data:
//   MPU-6050 (I2C) -> FlightCore.step() at 1 kHz on core 1 -> four PWM outputs
//   Bluepad32 (Bluetooth, core 0) -> sticks
//
// Controls (Xbox): right trigger = throttle, right stick = tilt (roll/pitch), left stick x = yaw,
//   Menu = arm (throttle at zero, drone level and still), B = kill (any time).
// Failsafe: controller silent for 250 ms, tilted past 75 deg for 0.3 s, or zero throttle for 5 s
// -> motors off. Battery below 3.5 V: controller rumbles, LED double-blinks; below 3.3 V: won't arm.
//
// Pin map: lite/ecad/MILLING.md (ESP32 MiniKit inner rows).
#include <Arduino.h>
#include <Bluepad32.h>
#include <Wire.h>

#include "lite/flight_core.hpp"

// ------------------------------------------------------------------ pins
// neuroflight motor order: 0 front-left, 1 front-right, 2 rear-right, 3 rear-left
static const int kMotorPin[4] = {17 /* M4 FL */, 16 /* M2 FR */, 19 /* M1 RR */, 26 /* M3 RL */};
static const int kLedPin = 18;
static const int kVbatPin = 36;          // ADC1_CH0, 100k / 47k divider
static const float kVbatDivider = (100.0f + 47.0f) / 47.0f;
static const int kSdaPin = 21, kSclPin = 22;
static const uint32_t kPwmHz = 20000;    // above hearing; the AO3400A switches this easily
static const int kPwmBits = 10;

// IMU mounting: sensor axes -> body FLU. VERIFY on the built board (BUILD.md step "IMU check"):
// flat on the desk the serial monitor must show acc z ~ +9.8; nose up -> pitch negative.
static const int8_t kImuToBody[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

// ------------------------------------------------------------------ MPU-6050
static const uint8_t kMpuAddr = 0x68;    // AD0 pulled low on the GY-521
static const float kGyroScale = (2000.0f / 32768.0f) * 0.017453293f;  // +-2000 dps -> rad/s
static const float kAccelScale = (8.0f / 32768.0f) * 9.81f;           // +-8 g -> m/s^2

static void mpu_write(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(kMpuAddr);
  Wire.write(reg);
  Wire.write(v);
  Wire.endTransmission();
}

static bool mpu_init() {
  mpu_write(0x6B, 0x80);  // reset
  delay(100);
  mpu_write(0x6B, 0x01);  // wake, PLL on gyro X
  mpu_write(0x19, 0x00);  // sample rate = 1 kHz (with DLPF on)
  mpu_write(0x1A, 0x02);  // DLPF 98 Hz gyro / 94 Hz accel
  mpu_write(0x1B, 0x18);  // gyro +-2000 dps
  mpu_write(0x1C, 0x10);  // accel +-8 g
  Wire.beginTransmission(kMpuAddr);
  Wire.write(0x75);       // WHO_AM_I
  Wire.endTransmission(false);
  if (Wire.requestFrom(kMpuAddr, (uint8_t)1) != 1) return false;
  const uint8_t who = Wire.read();
  return who == 0x68 || who == 0x70 || who == 0x72;  // MPU-6050 and common clones
}

static bool mpu_read(lite::ImuSample& s) {
  Wire.beginTransmission(kMpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(kMpuAddr, (uint8_t)14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; ++i) {
    const uint8_t hi = Wire.read(), lo = Wire.read();
    r[i] = (int16_t)((hi << 8) | lo);
  }
  const float a[3] = {r[0] * kAccelScale, r[1] * kAccelScale, r[2] * kAccelScale};
  const float g[3] = {r[4] * kGyroScale, r[5] * kGyroScale, r[6] * kGyroScale};  // r[3] = temperature
  float ab[3], gb[3];
  for (int i = 0; i < 3; ++i) {
    ab[i] = kImuToBody[i][0] * a[0] + kImuToBody[i][1] * a[1] + kImuToBody[i][2] * a[2];
    gb[i] = kImuToBody[i][0] * g[0] + kImuToBody[i][1] * g[1] + kImuToBody[i][2] * g[2];
  }
  s.accel = nf::Vec3{ab[0], ab[1], ab[2]};
  s.gyro = nf::Vec3{gb[0], gb[1], gb[2]};
  return true;
}

// ------------------------------------------------------------------ motors
static void motors_init() {
  for (int i = 0; i < 4; ++i) {
    pinMode(kMotorPin[i], OUTPUT);
    digitalWrite(kMotorPin[i], LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(kMotorPin[i], kPwmHz, kPwmBits);
#else
    ledcSetup(i, kPwmHz, kPwmBits);
    ledcAttachPin(kMotorPin[i], i);
#endif
  }
}

static void motor_write(int i, float duty) {
  const uint32_t d = (uint32_t)(nf::clampf(duty, 0.0f, 1.0f) * ((1 << kPwmBits) - 1) + 0.5f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(kMotorPin[i], d);
#else
  ledcWrite(i, d);
#endif
}

// ------------------------------------------------------------------ shared state
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static lite::Sticks g_sticks;            // written by loop() (Bluepad32), read by the control task
static uint32_t g_last_packet_ms = 0;
static lite::CoreOutput g_out;           // written by the control task, read by loop()
static float g_vbat = 4.0f;
static lite::ImuSample g_imu;
static volatile uint32_t g_overruns = 0, g_imu_errors = 0;
static volatile int g_spin_motor = -1;   // bench test: motor index, or -1
static volatile float g_spin_duty = 0.0f;
static volatile uint32_t g_spin_until = 0;

static lite::FlightCore g_core;
static ControllerPtr g_ctl = nullptr;

// ------------------------------------------------------------------ control task (core 1, 1 kHz)
static void control_task(void*) {
  TickType_t wake = xTaskGetTickCount();
  float vbat_f = 4.0f;
  uint32_t n = 0;
  for (;;) {
    lite::ImuSample imu;
    if (!mpu_read(imu)) {
      ++g_imu_errors;
      imu = g_imu;                       // hold the last sample for one tick
    }
    const uint32_t now = millis();
    if ((n++ & 63) == 0) {               // battery at ~16 Hz, filtered
      const float v = analogReadMilliVolts(kVbatPin) * 0.001f * kVbatDivider;
      vbat_f += 0.2f * (v - vbat_f);
    }
    lite::Sticks st;
    portENTER_CRITICAL(&g_mux);
    st = g_sticks;
    st.age_ms = now - g_last_packet_ms;
    portEXIT_CRITICAL(&g_mux);

    const lite::CoreOutput out = g_core.step(imu, st, vbat_f, now);
    const bool spin = out.state != lite::State::ARMED && g_spin_motor >= 0 && (int32_t)(g_spin_until - now) > 0;
    for (int i = 0; i < 4; ++i) motor_write(i, spin ? (i == g_spin_motor ? g_spin_duty : 0.0f) : out.motor[i]);
    if (!spin) g_spin_motor = -1;

    portENTER_CRITICAL(&g_mux);
    g_out = out;
    g_vbat = vbat_f;
    g_imu = imu;
    portEXIT_CRITICAL(&g_mux);

    if (xTaskDelayUntil(&wake, 1) == pdFALSE) ++g_overruns;  // 1 tick = 1 ms
  }
}

// ------------------------------------------------------------------ Bluepad32
static void on_connected(ControllerPtr ctl) {
  if (g_ctl == nullptr) {
    g_ctl = ctl;
    Serial.println("controller connected");
  } else {
    ctl->disconnect();  // one pilot
  }
}

static void on_disconnected(ControllerPtr ctl) {
  if (ctl == g_ctl) {
    g_ctl = nullptr;
    portENTER_CRITICAL(&g_mux);
    g_sticks = lite::Sticks{};           // connected = false -> failsafe
    portEXIT_CRITICAL(&g_mux);
    Serial.println("controller disconnected");
  }
}

static const uint16_t kMiscStart = 0x04;  // Bluepad32 MISC_BUTTON_START (Xbox "Menu")

static void read_controller() {
  if (g_ctl == nullptr || !g_ctl->isConnected() || !g_ctl->hasData()) return;
  lite::Sticks s;
  s.connected = true;
  s.throttle = g_ctl->throttle() / 1023.0f;  // right trigger
  s.roll = g_ctl->axisRX() / 512.0f;
  s.pitch = -g_ctl->axisRY() / 512.0f;       // stick up is negative in Bluepad32
  s.yaw = g_ctl->axisX() / 512.0f;
  s.arm = (g_ctl->miscButtons() & kMiscStart) != 0;
  s.kill = g_ctl->b();
  portENTER_CRITICAL(&g_mux);
  g_sticks = s;
  g_last_packet_ms = millis();
  portEXIT_CRITICAL(&g_mux);
}

// ------------------------------------------------------------------ LED, rumble, serial
static void led_update(const lite::CoreOutput& o, uint32_t now) {
  bool on;
  if (o.state == lite::State::CALIBRATING) on = (now / 60) & 1;               // fast flicker: keep still
  else if (o.state == lite::State::ARMED) on = !(o.flags & lite::BATTERY_LOW) || ((now / 150) % 4 < 3);
  else if (o.flags & lite::NO_CONTROLLER) on = (now % 2000) < 100;            // short blip: pair the controller
  else if (o.flags & lite::BATTERY_LOW) on = (now % 1000) < 100 || ((now % 1000) > 200 && (now % 1000) < 300);
  else on = (now / 500) & 1;                                                  // slow blink: ready to arm
  digitalWrite(kLedPin, on ? HIGH : LOW);
}

static void serial_commands() {
  // bench test (BUILD.md): "spin <motor 0-3> <duty 0-1>" runs one motor for 3 s while disarmed.
  // Props off for the direction check, on (and the drone clamped to the scale) for thrust.
  static char buf[32];
  static int n = 0;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[n] = 0;
      int m = -1;
      float d = 0;
      if (n > 0 && sscanf(buf, "spin %d %f", &m, &d) == 2 && m >= 0 && m < 4) {
        g_spin_duty = nf::clampf(d, 0.0f, 1.0f);
        g_spin_until = millis() + 3000;
        g_spin_motor = m;
        Serial.printf("spinning motor %d at %.2f for 3 s\n", m, g_spin_duty);
      }
      n = 0;
    } else if (n < (int)sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(kLedPin, OUTPUT);
  motors_init();
  for (int i = 0; i < 4; ++i) motor_write(i, 0.0f);
  analogSetPinAttenuation(kVbatPin, ADC_11db);
  Wire.begin(kSdaPin, kSclPin, 400000);
  if (!mpu_init()) {
    Serial.println("MPU-6050 not found: check SDA 21 / SCL 22 / 3V3 / AD0");
    for (;;) { digitalWrite(kLedPin, (millis() / 100) & 1); delay(10); }
  }
  g_core.init();
  BP32.setup(&on_connected, &on_disconnected);
  BP32.enableVirtualDevice(false);
  // control loop on core 1 above loop() (Bluetooth runs on core 0)
  xTaskCreatePinnedToCore(control_task, "ctrl", 6144, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
  Serial.println("NF-3D Lite ready: keep still for calibration, then pair the Xbox controller");
}

void loop() {
  if (BP32.update()) read_controller();
  lite::CoreOutput o;
  float vbat;
  lite::ImuSample imu;
  portENTER_CRITICAL(&g_mux);
  o = g_out;
  vbat = g_vbat;
  imu = g_imu;
  portEXIT_CRITICAL(&g_mux);
  const uint32_t now = millis();
  led_update(o, now);
  serial_commands();

  static uint32_t last_rumble = 0;
  if (g_ctl && (o.flags & lite::BATTERY_LOW) && now - last_rumble > 10000) {
    g_ctl->playDualRumble(0, 400, 0x80, 0x40);
    last_rumble = now;
  }
  static uint32_t last_print = 0;
  if (now - last_print > 500) {
    last_print = now;
    static const char* names[] = {"CALIBRATING", "DISARMED", "ARMED"};
    Serial.printf("%s flags 0x%03lx vbat %.2f roll %.1f pitch %.1f acc %.1f %.1f %.1f overruns %lu imu_err %lu\n",
                  names[(int)o.state], (unsigned long)o.flags, vbat, o.q.roll() * 57.3f, o.q.pitch() * 57.3f,
                  imu.accel.x, imu.accel.y, imu.accel.z, (unsigned long)g_overruns, (unsigned long)g_imu_errors);
  }
  delay(2);
}

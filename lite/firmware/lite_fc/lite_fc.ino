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
// v3 (companion board fitted): A = position hold, X = angle mode (manual), Y = AUTO (laptop);
//   in the hold modes the right stick is velocity, the left stick y climbs, the trigger is ignored.
// Failsafe: angle mode: controller silent for 250 ms, tilted past 75 deg for 0.3 s, or zero throttle
// for 5 s -> motors off. Hold modes: controller silent -> LAND; flow lost -> ALT_HOLD; height lost
// -> LAND; laptop silent in AUTO -> hover, then POS_HOLD. Battery below 3.5 V: controller rumbles,
// LED double-blinks; below 3.3 V: won't arm. In flight, under 3.3 V for 2 s: lands (hold modes land
// themselves; in angle mode the trigger is capped so it sinks); under 3.0 V for 1 s: motors off.
// IMU reads failing for 20 ms: motors off.
//
// Companion (XIAO ESP32-S3 Sense, v3): UART2 at 115200, frames of lite/autonomy.hpp (link::).
// It sends SENS (PMW3901 flow + VL53L1X range, 100 Hz) and relays CMD from the laptop; this
// board sends TELEM at 25 Hz. Without the companion the board flies exactly like v2.
//
// Pin map: lite/ecad/MILLING.md (ESP32 MiniKit inner rows).
#include <Arduino.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <esp_system.h>

#include <lite_fc.h>  // library discovery (see lite_fc.h)
#include "lite/autonomy.hpp"
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
// companion UART: ESP32 MiniKit outer-row pins, wired by hand (BUILD.md "v3 wiring"); any free
// GPIO works (25, 27, 32, 33), change these two if you solder elsewhere
static const int kCompRxPin = 27;        // <- XIAO D6 (TX)
static const int kCompTxPin = 25;        // -> XIAO D7 (RX)
static const uint32_t kCompBaud = 115200;

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
static volatile uint32_t g_sens_frames = 0, g_cmd_frames = 0;
static uint16_t g_cmd_seq = 0;
static volatile int g_spin_motor = -1;   // bench test: motor index, or -1
static volatile float g_spin_duty = 0.0f;
static volatile uint32_t g_spin_until = 0;

static lite::FlightCore g_core;
static ControllerPtr g_ctl = nullptr;

// ------------------------------------------------------------------ companion link (read on core 1)
static lite::link::Parser g_parser;

// Drain the UART; fill aux with the newest SENS / CMD frames.
static void companion_poll(lite::AuxInput& aux, uint32_t now, uint32_t& last_cmd_ms) {
  aux.sample_new = false;
  aux.comp.cmd_new = false;
  int budget = 128;  // bytes per tick (115200 baud = 11.5 bytes/ms)
  while (budget-- > 0 && Serial2.available() > 0) {
    if (!g_parser.push((uint8_t)Serial2.read())) continue;
    if (g_parser.type() == lite::link::SENS) {
      lite::link::Sens s;
      if (!lite::link::decode_sens(g_parser.payload(), g_parser.len(), s)) continue;
      aux.sample_new = true;
      aux.flow.flow_x = s.flow_x_mrad * 0.001f;
      aux.flow.flow_y = s.flow_y_mrad * 0.001f;
      aux.flow.quality = s.quality / 255.0f;
      aux.flow.valid = (s.flags & lite::link::FLOW_OK) != 0;
      aux.range.range_m = s.range_mm * 0.001f;
      aux.range.valid = (s.flags & lite::link::RANGE_OK) != 0;
      ++g_sens_frames;
    } else if (g_parser.type() == lite::link::CMD) {
      lite::link::Cmd c;
      if (!lite::link::decode_cmd(g_parser.payload(), g_parser.len(), c)) continue;
      aux.comp.cmd = c;
      aux.comp.cmd_new = true;
      last_cmd_ms = now;
      ++g_cmd_frames;
    }
  }
  aux.comp.cmd_age_s = (now - last_cmd_ms) * 0.001f;
}

// ------------------------------------------------------------------ control task (core 1, 1 kHz)
static void control_task(void*) {
  TickType_t wake = xTaskGetTickCount();
  float vbat_f = 4.0f;
  uint32_t n = 0;
  lite::AuxInput aux;
  uint32_t last_cmd_ms = 0;
  bool companion_seen = false;
  for (;;) {
    lite::ImuSample imu;
    if (!mpu_read(imu)) {
      ++g_imu_errors;
      imu = g_imu;                       // repeat the last sample, marked stale: the core
      imu.valid = false;                 // disarms after ArmingConfig::imu_timeout_ms of these
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

    companion_poll(aux, now, last_cmd_ms);
    companion_seen |= aux.sample_new;
    const lite::CoreOutput out = g_core.step(imu, st, vbat_f, now, companion_seen ? &aux : nullptr);
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
  s.climb = -g_ctl->axisY() / 512.0f;        // left stick up = climb (hold modes)
  s.btn_hold = g_ctl->a();
  s.btn_angle = g_ctl->x();
  s.btn_auto = g_ctl->y();
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

static void send_telemetry(const lite::CoreOutput& o, float vbat) {
  lite::link::Telem t;
  t.mode = (uint8_t)o.mode;
  t.state = (uint8_t)o.state;
  t.flags = (uint16_t)o.flags;
  const float p[3] = {o.pos.x, o.pos.y, o.pos.z}, v[3] = {o.vel.x, o.vel.y, o.vel.z};
  for (int i = 0; i < 3; ++i) {
    t.pos_cm[i] = (int16_t)nf::clampf(p[i] * 100.0f, -32000.0f, 32000.0f);
    t.vel_cms[i] = (int16_t)nf::clampf(v[i] * 100.0f, -32000.0f, 32000.0f);
  }
  t.yaw_mrad = (int16_t)(o.q.yaw() * 1000.0f);
  t.vbat_mv = (uint16_t)(vbat * 1000.0f);
  t.cmd_seq = g_cmd_seq;
  uint8_t buf[lite::link::kTelemLen + lite::link::kOverhead];
  const size_t len = lite::link::encode_telem(t, buf);
  if (Serial2.availableForWrite() >= (int)len) Serial2.write(buf, len);
}

void setup() {
  Serial.begin(115200);
  // a brown-out here means the 3.3 V rail dipped under motor load (pack sag): see BUILD.md, brown-out check
  if (esp_reset_reason() == ESP_RST_BROWNOUT) Serial.println("WARNING: last reset was a BROWN-OUT (supply dipped)");
  Serial2.begin(kCompBaud, SERIAL_8N1, kCompRxPin, kCompTxPin);
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

  static uint32_t last_telem = 0;
  if (now - last_telem >= 40) {   // 25 Hz to the companion (the laptop's view of the drone)
    last_telem = now;
    g_cmd_seq = g_core.autopilot().last_cmd_seq();
    send_telemetry(o, vbat);
  }
  static uint32_t last_refuse = 0;
  if (g_ctl && o.refused && now - last_refuse > 500) {  // mode button refused: short buzz
    g_ctl->playDualRumble(0, 150, 0xC0, 0x00);
    last_refuse = now;
  }
  static uint32_t last_rumble = 0;
  const uint32_t rumble_every = (o.flags & lite::BATTERY_LAND) ? 2000 : 10000;  // landing: every 2 s
  if (g_ctl && (o.flags & (lite::BATTERY_LOW | lite::BATTERY_LAND)) && now - last_rumble > rumble_every) {
    g_ctl->playDualRumble(0, 400, 0x80, 0x40);
    last_rumble = now;
  }
  static uint32_t last_print = 0;
  if (now - last_print > 500) {
    last_print = now;
    static const char* names[] = {"CALIBRATING", "DISARMED", "ARMED"};
    Serial.printf("%s %s flags 0x%03lx vbat %.2f roll %.1f pitch %.1f acc %.1f %.1f %.1f overruns %lu imu_err %lu\n",
                  names[(int)o.state], lite::mode_name(o.mode), (unsigned long)o.flags, vbat, o.q.roll() * 57.3f,
                  o.q.pitch() * 57.3f, imu.accel.x, imu.accel.y, imu.accel.z, (unsigned long)g_overruns,
                  (unsigned long)g_imu_errors);
    if (g_sens_frames)
      Serial.printf("  companion: sens %lu cmd %lu crc_err %lu | pos %.2f %.2f %.2f m  h_ok %d flow_ok %d\n",
                    (unsigned long)g_sens_frames, (unsigned long)g_cmd_frames, (unsigned long)g_parser.crc_errors(),
                    o.pos.x, o.pos.y, o.pos.z, o.height_valid, o.flow_valid);
  }
  delay(2);
}

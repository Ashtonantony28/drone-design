// NF-3D Lite flight core: everything between "IMU sample + Xbox sticks" and
// "four motor duties", with no hardware calls, so it runs and is tested on a PC.
//
// Loop (call step() at kRateLoopHz = 1 kHz):
//   gyro: at-rest calibration (nf::GyroCalibrator), 150 Hz biquad low-pass
//   attitude: nf::Mahony on gyro + accelerometer (no flow/baro: accel reference = gravity)
//   500 Hz: angle mode (same law as lite/fly.py AngleMode) -> body-rate setpoint, collective
//   1 kHz: nf::RatePid -> nf::Mixer -> motor commands in [0, 1]
//   arming / failsafe state machine (Arming below)
//
// Body frame FLU (x forward, y left, z up). Motor order (neuroflight):
//   0 front-left, 1 front-right, 2 rear-right, 3 rear-left.
// Positive roll = right side down; positive pitch = nose down.
#pragma once

#include <cstdint>

#include "nf/control.hpp"
#include "nf/estimator.hpp"
#include "nf/filters.hpp"
#include "nf/math.hpp"
#include "nf/vehicle_params.hpp"

namespace lite {

// Pilot input, already normalised by the gamepad layer.
struct Sticks {
  float throttle = 0.0f;  // [0, 1]  right trigger
  float roll = 0.0f;      // [-1, 1] right stick x, + = right
  float pitch = 0.0f;     // [-1, 1] right stick y, + = forward (nose down)
  float yaw = 0.0f;       // [-1, 1] left stick x,  + = clockwise seen from above
  bool arm = false;       // arm button held (Menu)
  bool kill = false;      // kill button held (B)
  bool connected = false;
  uint32_t age_ms = 0;    // time since the last controller packet
};

struct StickConfig {
  float deadband = 0.06f;
  float expo = 0.3f;              // 0 linear, 1 cubic
  float max_tilt_rad = 0.52f;     // 30 deg at full stick
  float max_yaw_rate = 3.0f;      // rad/s at full stick
};

// Desired attitude / thrust from the sticks.
struct Setpoint {
  float roll = 0.0f, pitch = 0.0f;  // rad
  float yaw_rate = 0.0f;            // rad/s about body z (FLU: + = counter-clockwise)
  float thrust_n = 0.0f;            // collective before tilt compensation
};

Setpoint map_sticks(const Sticks& s, const StickConfig& c = StickConfig{});

struct AngleConfig {
  float kp[2] = {nf::vehicle::kAngleKp[0], nf::vehicle::kAngleKp[1]};  // 1/s
  nf::Vec3 rate_limit{nf::vehicle::kMaxBodyRate[0], nf::vehicle::kMaxBodyRate[1], nf::vehicle::kMaxBodyRate[2]};
  float thrust_max_total = 4.0f * nf::vehicle::kThrustMax;
};

// Angle mode, the law simulated in lite/fly.py: tilt the body z axis onto the
// desired one along the shortest arc (exactly inverted: roll about body x),
// P gain to a roll/pitch rate setpoint; yaw rate straight from the stick.
// Collective is tilt-compensated (cos clamped at 0.5); when upside down it
// drops to 30 % so the flip-over does not drive the drone into the floor.
nf::CtbrCommand angle_mode(const nf::Quat& q_est, const Setpoint& sp, const AngleConfig& c = AngleConfig{});

// ------------------------------------------------------------------ arming
enum class State : uint8_t { CALIBRATING, DISARMED, ARMED };

enum Flag : uint32_t {
  NO_CONTROLLER = 1u << 0,
  THROTTLE_UP = 1u << 1,
  NOT_LEVEL = 1u << 2,
  GYRO_CAL = 1u << 3,
  BATTERY_EMPTY = 1u << 4,
  BATTERY_LOW = 1u << 5,     // warning only (rumble / LED)
  CRASH = 1u << 6,           // disarmed: tilted past crash_tilt for crash_ms
  LINK_LOST = 1u << 7,       // disarmed: controller silent for link_timeout_ms
  KILLED = 1u << 8,          // disarmed by the kill button
  IDLE_TIMEOUT = 1u << 9,    // disarmed: armed at zero throttle for idle_disarm_ms
};

struct ArmingConfig {
  float arm_max_throttle = 0.05f;
  float arm_max_tilt = 0.44f;      // 25 deg
  float crash_tilt = 1.31f;        // 75 deg
  uint32_t crash_ms = 300;
  uint32_t link_timeout_ms = 250;  // brushed micro: cutting the motors is the safe failsafe
  uint32_t idle_disarm_ms = 5000;
  float vbat_empty = 3.30f;        // refuse to arm below this (V, resting)
  float vbat_low = 3.50f;          // warn below this
};

struct ArmingInputs {
  bool gyro_cal_done = false;
  float tilt = 0.0f;   // rad, from the attitude estimate
  float vbat = 4.2f;   // V
  Sticks sticks{};
  uint32_t now_ms = 0;
};

class Arming {
 public:
  void init(const ArmingConfig& c = ArmingConfig{}) { c_ = c; }
  State update(const ArmingInputs& in);
  State state() const { return state_; }
  uint32_t flags() const { return flags_; }      // current refusal / warning reasons
  uint32_t last_disarm() const { return last_disarm_; }

 private:
  void disarm(uint32_t why);
  ArmingConfig c_{};
  State state_ = State::CALIBRATING;
  uint32_t flags_ = 0, last_disarm_ = 0;
  uint32_t tilt_since_ = 0, idle_since_ = 0;
  bool tilt_high_ = false, idle_ = false, arm_prev_ = true;  // arm must be released once after boot
};

// ------------------------------------------------------------------ the core
struct ImuSample {
  nf::Vec3 gyro{};   // rad/s, body FLU (already rotated from the sensor frame)
  nf::Vec3 accel{};  // m/s^2 specific force, body FLU (level at rest: [0, 0, +g])
};

struct CoreOutput {
  float motor[4] = {0, 0, 0, 0};  // duty in [0, 1], neuroflight motor order
  State state = State::CALIBRATING;
  uint32_t flags = 0;
  float tilt = 0.0f;              // rad
  nf::Quat q{};                   // attitude estimate
  bool saturated = false;
};

class FlightCore {
 public:
  void init();
  CoreOutput step(const ImuSample& imu, const Sticks& sticks, float vbat, uint32_t now_ms);
  const nf::Mahony& mahony() const { return mahony_; }
  StickConfig sticks_cfg{};
  AngleConfig angle_cfg{};

 private:
  nf::GyroCalibrator cal_{};
  nf::Mahony mahony_{};
  nf::BiquadLpfVec3 gyro_lpf_{};
  nf::RatePid pid_{};
  nf::Mixer mixer_{};
  Arming arming_{};
  nf::CtbrCommand cmd_{};
  uint32_t tick_ = 0;
  bool attitude_seeded_ = false;
  float dt_ = 1.0f / nf::vehicle::kRateLoopHz;
  int outer_div_ = nf::vehicle::kRateLoopHz / nf::vehicle::kControlRateHz;
};

}  // namespace lite

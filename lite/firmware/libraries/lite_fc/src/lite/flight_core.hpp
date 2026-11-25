// NF-3D Lite flight core: everything between "IMU sample + Xbox sticks" and
// "four motor duties", with no hardware calls, so it runs and is tested on a PC.
//
// Loop (call step() at kRateLoopHz = 1 kHz):
//   gyro: at-rest calibration (nf::GyroCalibrator), 150 Hz biquad low-pass
//   attitude: nf::Mahony on gyro + accelerometer (no flow/baro: accel reference = gravity)
//   500 Hz: angle mode (same law as lite/fly.py AngleMode) -> body-rate setpoint, collective
//   1 kHz: nf::RatePid -> nf::Mixer -> motor commands in [0, 1]
//   arming / failsafe state machine (Arming below)
//   v3 (flow + ToF via the companion board, AuxInput): nf::TranslationKf at 1 kHz with
//   corrections per sensor frame, KF-referenced Mahony, and the Autopilot of
//   lite/autonomy.hpp (position hold, altitude hold, land, AUTO) in place of angle mode.
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
#include "lite/autonomy.hpp"

namespace lite {

// Pilot input, already normalised by the gamepad layer.
struct Sticks {
  float throttle = 0.0f;  // [0, 1]  right trigger
  float roll = 0.0f;      // [-1, 1] right stick x, + = right
  float pitch = 0.0f;     // [-1, 1] right stick y, + = forward (nose down)
  float yaw = 0.0f;       // [-1, 1] left stick x,  + = clockwise seen from above
  bool arm = false;       // arm button held (Menu)
  bool kill = false;      // kill button held (B)
  float climb = 0.0f;     // [-1, 1] left stick y, + = up (hold modes)
  bool btn_hold = false;  // A held: position hold
  bool btn_angle = false; // X held: angle mode (manual)
  bool btn_auto = false;  // Y held: AUTO (laptop commands)
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
  LANDED = 1u << 10,         // disarmed: the LAND mode's landing detector fired
  IMU_FAIL = 1u << 11,       // IMU reads failing for imu_timeout_ms: won't arm; disarms in flight
  BATTERY_LAND = 1u << 12,   // in flight, under vbat_land for vbat_land_ms: lands (latched until disarm)
  BATTERY_CUTOFF = 1u << 13, // disarmed: under vbat_cutoff for vbat_cutoff_ms (protects the pack)
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
  // In flight (V under load). The pack has no protection board of its own once JP1 is bridged,
  // so the firmware is the over-discharge protection.
  float vbat_land = 3.30f;         // below this for vbat_land_ms: land
  uint32_t vbat_land_ms = 2000;
  float vbat_cutoff = 3.00f;       // below this for vbat_cutoff_ms: motors off
  uint32_t vbat_cutoff_ms = 1000;
  // Landing without a height sensor (angle mode): the trigger is capped at a descent-rate hold,
  // cap = base + land_kv * (land_vz - vz), where vz is the accelerometer's vertical speed (leaky,
  // land_vz_tau, so accel bias costs at most bias * tau) and base starts at land_ceiling_frac of
  // the recent average trigger and falls by land_ceiling_rate per s (so it lands even with bias).
  float land_ceiling_frac = 0.98f;
  float land_ceiling_rate = 0.03f;
  float land_vz = -0.5f;      // m/s
  float land_kv = 0.3f;      // trigger per m/s
  float land_vz_tau = 4.0f;   // s
  uint32_t imu_timeout_ms = 20;    // IMU reads failing this long: disarm (the attitude is unknown)
};

struct ArmingInputs {
  bool gyro_cal_done = false;
  float tilt = 0.0f;   // rad, from the attitude estimate
  float vbat = 4.2f;   // V
  Sticks sticks{};
  uint32_t now_ms = 0;
  bool hold_mode = false;  // an autopilot mode flies the drone: link loss lands (Autopilot),
                           // and zero trigger is not idle
  bool imu_ok = true;      // false: the IMU has failed (refuses arming)
};

class Arming {
 public:
  void init(const ArmingConfig& c = ArmingConfig{}) { c_ = c; }
  State update(const ArmingInputs& in);
  State state() const { return state_; }
  uint32_t flags() const { return flags_; }      // current refusal / warning reasons
  uint32_t last_disarm() const { return last_disarm_; }
  void force_disarm(uint32_t why) { if (state_ == State::ARMED) disarm(why); }

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
  bool valid = true; // false: this read failed and the sample is the last good one repeated
};

// v3 sensors and commands from the companion board (all optional).
struct AuxInput {
  bool sample_new = false;   // a SENS frame arrived since the last step
  nf::FlowSample flow{};
  nf::RangeSample range{};
  CompanionInput comp{};     // laptop command (relayed by the companion)
};

struct CoreOutput {
  float motor[4] = {0, 0, 0, 0};  // duty in [0, 1], neuroflight motor order
  State state = State::CALIBRATING;
  uint32_t flags = 0;
  float tilt = 0.0f;              // rad
  nf::Quat q{};                   // attitude estimate
  bool saturated = false;
  Mode mode = Mode::ANGLE;
  nf::Vec3 pos{}, vel{};          // KF estimate
  bool height_valid = false, flow_valid = false;
  bool refused = false;           // a mode button was refused (rumble)
};

class FlightCore {
 public:
  void init();
  CoreOutput step(const ImuSample& imu, const Sticks& sticks, float vbat, uint32_t now_ms,
                  const AuxInput* aux = nullptr);
  const nf::Mahony& mahony() const { return mahony_; }
  const nf::TranslationKf& kf() const { return kf_; }
  const Autopilot& autopilot() const { return ap_; }
  StickConfig sticks_cfg{};
  AngleConfig angle_cfg{};
  ArmingConfig arming_cfg{};

 private:
  nf::GyroCalibrator cal_{};
  nf::Mahony mahony_{};
  nf::BiquadLpfVec3 gyro_lpf_{};
  nf::RatePid pid_{};
  nf::Mixer mixer_{};
  Arming arming_{};
  nf::CtbrCommand cmd_{};
  // v3 estimator + autopilot
  struct Age {  // time since the KF last accepted a measurement (exact once past the KF's 0.2 s)
    float age = 1e3f;
    bool was_valid = false;
    void update(bool valid, float dt) {
      if (valid) { age = 0.0f; } else { age = was_valid ? 0.2f : age + dt; }
      was_valid = valid;
    }
  };
  nf::Vec3 accel_ref() const;
  nf::TranslationKf kf_{};
  nf::Pt1Vec3 acc_lpf_{};
  nf::Pt1 acc_ref_[3];
  nf::Vec3 prev_kf_vel_{};
  Age flow_age_{}, height_age_{};
  Autopilot ap_{};
  AutopilotOutput ap_out_{};
  bool btn_prev_[3] = {true, true, true};  // buttons must be released once after boot
  bool btn_edge_[3] = {false, false, false};
  bool armed_prev_ = false;
  CompanionInput comp_{};  // latched between outer-loop steps (a CMD frame is never dropped)
  uint32_t tick_ = 0;
  bool attitude_seeded_ = false;
  // failsafes: IMU reads failing, battery under the land / cut-off voltage (ArmingConfig)
  bool imu_bad_ = false, vlow_ = false, vcut_ = false, batt_land_ = false;
  uint32_t imu_bad_since_ = 0, vlow_since_ = 0, vcut_since_ = 0;
  float thr_avg_ = 0.0f, ceiling_ = 1.0f, land_vz_ = 0.0f;
  float dt_ = 1.0f / nf::vehicle::kRateLoopHz;
  int outer_div_ = nf::vehicle::kRateLoopHz / nf::vehicle::kControlRateHz;
};

}  // namespace lite

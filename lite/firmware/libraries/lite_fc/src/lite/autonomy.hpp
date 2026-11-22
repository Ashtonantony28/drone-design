// NF-3D Lite v3 autonomy: position hold on optical flow + ToF, flight modes,
// failsafes, and the companion (XIAO ESP32-S3) serial link. No hardware calls.
//
//   PositionHold   position P -> velocity PI -> acceleration -> tilt + collective.
//                  Exact twin of lite/autonomy.py PosHoldLaw (fixture: tests/pos_fixture.hpp).
//   Autopilot      flight modes and their failsafes (lite/autonomy.py Autopilot, plus the
//                  pilot / companion inputs the simulator does not model).
//   link::         framed UART protocol between the XIAO and the flight board (lite/link.hpp).
//
// Frames: world ENU-like (x, y horizontal from the KF reset point, z = height above
// the floor, up); "heading frame" = forward / left / up in the current yaw.
#pragma once

#include <cstddef>
#include <cstdint>

#include "nf/math.hpp"
#include "nf/vehicle_params.hpp"
#include "lite/link.hpp"

namespace lite {

// ------------------------------------------------------------------ PositionHold
enum class LawMode : uint8_t { POS = 0, ALT = 1, LAND = 2 };  // values = lite/autonomy.py MODE_*

struct PosHoldConfig {
  float kp_pos[3] = {nf::vehicle::kPosKp[0], nf::vehicle::kPosKp[1], nf::vehicle::kPosKp[2]};
  float kp_vel[3] = {nf::vehicle::kVelKp[0], nf::vehicle::kVelKp[1], nf::vehicle::kVelKp[2]};
  float ki_vel[3] = {nf::vehicle::kVelKi[0], nf::vehicle::kVelKi[1], nf::vehicle::kVelKi[2]};
  float i_limit[3] = {nf::vehicle::kVelILimit[0], nf::vehicle::kVelILimit[1], nf::vehicle::kVelILimit[2]};
  float v_max[3] = {nf::vehicle::kVelMax[0], nf::vehicle::kVelMax[1], nf::vehicle::kVelMax[2]};
  float a_max_xy = nf::vehicle::kAccMaxXy;
  float max_tilt = nf::vehicle::kPosMaxTilt;
  float hover_n = nf::vehicle::kMass * nf::vehicle::kGravity;
  float land_speed = 0.4f;  // m/s
};

struct TiltThrust {
  float roll = 0.0f, pitch = 0.0f;  // rad, FLU (+roll right side down, +pitch nose down)
  float thrust_n = 0.0f;            // collective before tilt compensation
};

class PositionHold {
 public:
  static constexpr float kXyCoastS = 0.5f;   // control x/y on the KF prediction through gaps this short
  static constexpr float kXyForgetS = 1.0f;  // after this long without flow the hold point follows the drone
  static constexpr float kG = 9.81f;
  static constexpr float kLeash = 0.3f;      // m, hold point at most this far from the drone, per axis

  void init(const PosHoldConfig& c = PosHoldConfig{}) { c_ = c; }
  void reset(const nf::Vec3& p) { integ_ = {}; p_sp_ = p; }
  // p, v: KF estimate; yaw: attitude yaw (rad); v_cmd: world velocity command (0 = hold);
  // flow_age / z_valid: estimator health. dt: outer-loop period.
  TiltThrust step(const nf::Vec3& p, const nf::Vec3& v, float yaw, const nf::Vec3& v_cmd, LawMode mode, float flow_age,
                  bool z_valid, float dt);
  const nf::Vec3& setpoint() const { return p_sp_; }
  const nf::Vec3& integrator() const { return integ_; }
  const PosHoldConfig& config() const { return c_; }

 private:
  PosHoldConfig c_{};
  nf::Vec3 integ_{}, p_sp_{};
};

// ------------------------------------------------------------------ Autopilot
enum class Mode : uint8_t { ANGLE = 0, POS_HOLD = 1, ALT_HOLD = 2, LAND = 3, AUTO = 4 };
const char* mode_name(Mode m);

struct PilotInput {          // from the Xbox controller (already deadbanded / shaped)
  bool link_ok = false;
  bool req_hold = false;     // A (edge)
  bool req_angle = false;    // X (edge)
  bool req_auto = false;     // Y (edge)
  float throttle = 0.0f;     // right trigger [0, 1]
  float fwd = 0.0f, left = 0.0f, up = 0.0f;  // [-1, 1] velocity sticks (hold modes)
  float tilt_roll = 0.0f, tilt_pitch = 0.0f; // rad, ALT_HOLD lets the pilot tilt
  float yaw_rate = 0.0f;     // rad/s (FLU, + = CCW)
};

struct CompanionInput {
  bool cmd_new = false;      // a CMD frame arrived this step
  link::Cmd cmd{};
  float cmd_age_s = 1e3f;    // since the last CMD frame
};

struct EstimateInput {
  nf::Vec3 p{}, v{};
  float yaw = 0.0f;
  float flow_age = 1e3f, height_age = 1e3f;  // s since the KF last accepted a flow / range update
};

struct AutopilotConfig {
  float cmd_v_max[3] = {nf::vehicle::kCmdVelMax[0], nf::vehicle::kCmdVelMax[1], nf::vehicle::kCmdVelMax[2]};
  float valid_age_s = 0.5f;         // flow / height older than this = lost (lite/autonomy.py)
  float min_engage_height = 0.15f;  // m, hold modes engage only in the air
  float angle_min_throttle = 0.20f; // X refused unless the trigger is at least here (no drop)
  float cmd_stale_s = 0.5f;         // AUTO: no companion command for this long -> hover in place
  float cmd_lost_s = 2.0f;          // AUTO: ... for this long -> POS_HOLD (pilot takes over)
  float max_auto_yaw_rate = 1.0f;   // rad/s
  float max_goto_m = 3.0f;          // per GOTO command, per axis
  float goto_kp = 1.0f;             // 1/s, GOTO distance -> velocity command
  float land_thrust_frac = 0.6f;    // landing detector: thrust < 60 % hover ...
  float land_detect_s = 0.5f;       // ... for this long -> disarm
  // blind landing (height lost): the KF vertical velocity freezes on the floor, so disarm after
  // the descent from the last known height should be over: h / land_speed * factor + margin.
  // factor 2: the KF velocity error frozen in at the loss can halve the real descent rate.
  float blind_land_factor = 2.0f;
  float blind_land_margin_s = 1.0f;
};

struct AutopilotOutput {
  Mode mode = Mode::ANGLE;
  bool use_law = false;        // false: ANGLE, the caller flies the pilot's sticks
  TiltThrust tt{};             // when use_law
  float yaw_rate = 0.0f;
  bool disarm_landed = false;  // landing detector fired
  bool refused = false;        // a mode request was refused this step (rumble)
};

class Autopilot {
 public:
  void init(const AutopilotConfig& c = AutopilotConfig{}, const PosHoldConfig& pc = PosHoldConfig{});
  // Call at the outer-loop rate while armed; call disarmed() when not armed.
  AutopilotOutput update(const EstimateInput& est, const PilotInput& pilot, const CompanionInput& comp, float dt);
  void disarmed() { mode_ = Mode::ANGLE; low_t_ = 0.0f; blind_t_ = 0.0f; }
  void request_land();
  Mode mode() const { return mode_; }
  bool hold_mode() const { return mode_ != Mode::ANGLE; }  // link loss lands instead of cutting the motors
  uint16_t last_cmd_seq() const { return last_seq_; }
  const PositionHold& law() const { return law_; }

 private:
  void engage(Mode m, const EstimateInput& est);
  AutopilotConfig c_{};
  PositionHold law_{};
  Mode mode_ = Mode::ANGLE;
  float low_t_ = 0.0f;
  float h_last_ = 0.0f, blind_t_ = 0.0f;
  nf::Vec3 auto_v_{};  // AUTO velocity command, heading frame
  nf::Vec3 goto_target_{};
  bool goto_active_ = false;
  float auto_yaw_ = 0.0f;
  uint16_t last_seq_ = 0;
};

}  // namespace lite

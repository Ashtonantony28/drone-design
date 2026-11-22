#include "lite/autonomy.hpp"

#include <cmath>

namespace lite {

using nf::Vec3;

// ------------------------------------------------------------------ PositionHold
// Line-for-line twin of lite/autonomy.py PosHoldLaw.step (float32 there too).
TiltThrust PositionHold::step(const Vec3& p, const Vec3& v, float yaw, const Vec3& v_cmd_in, LawMode mode,
                              float flow_age, bool z_valid, float dt) {
  const bool xy_valid = (flow_age < kXyCoastS) && z_valid;
  const bool land = mode == LawMode::LAND;
  Vec3 cmd = v_cmd_in;
  if (land) cmd.z = -c_.land_speed;
  const bool xy_on = (mode == LawMode::POS || land) && xy_valid;
  const bool z_on = z_valid || land;
  const bool mask[3] = {xy_on, xy_on, z_on};
  // hold point ("carrot"): a velocity command moves it at that speed, the position loop keeps
  // the drone on it (feed-forward + P); while commanded at most kLeash from the drone, with no
  // command it stays put; follows the drone on an axis that is not controlled
  const bool forget_xy = (flow_age > kXyForgetS) || !z_valid;
  const bool forget[3] = {forget_xy, forget_xy, !z_on};
  Vec3 v_d;
  for (int i = 0; i < 3; ++i) {
    float sp = p_sp_[i] + cmd[i] * dt;
    if (std::fabs(cmd[i]) > 1e-3f) sp = p[i] + nf::clampf(sp - p[i], -kLeash, kLeash);
    p_sp_[i] = forget[i] ? p[i] : sp;
    v_d[i] = cmd[i] + c_.kp_pos[i] * (p_sp_[i] - p[i]);
  }
  {
    const float nxy = std::sqrt(v_d.x * v_d.x + v_d.y * v_d.y);
    const float s = std::fmin(c_.v_max[0] / std::fmax(nxy, 1e-6f), 1.0f);
    v_d.x *= s;
    v_d.y *= s;
    v_d.z = nf::clampf(v_d.z, -c_.v_max[2], c_.v_max[2]);
  }
  const Vec3 e_v = v_d - v;
  Vec3 a;
  for (int i = 0; i < 3; ++i) {
    if (mask[i]) {
      integ_[i] = nf::clampf(integ_[i] + c_.ki_vel[i] * e_v[i] * dt, -c_.i_limit[i], c_.i_limit[i]);
    } else if (i < 2) {
      integ_[i] = 0.0f;  // x/y off: forget the wind estimate; z off (height unknown): keep the trim
    }
    a[i] = mask[i] ? c_.kp_vel[i] * e_v[i] + integ_[i] : 0.0f;
  }
  {
    const float na = std::sqrt(a.x * a.x + a.y * a.y);
    const float s = std::fmin(c_.a_max_xy / std::fmax(na, 1e-6f), 1.0f);
    a.x *= s;
    a.y *= s;
    a.z = nf::clampf(a.z, -0.5f * kG, 0.8f * kG);
  }
  const float c = std::cos(yaw), s = std::sin(yaw);
  const float af = c * a.x + s * a.y;
  const float al = -s * a.x + c * a.y;
  const float gz = kG + a.z;
  const float nrm = std::sqrt(af * af + al * al + gz * gz);
  const float dl = al / nrm, df = af / nrm, dz = gz / nrm;
  TiltThrust out;
  out.roll = nf::clampf(std::asin(nf::clampf(-dl, -1.0f, 1.0f)), -c_.max_tilt, c_.max_tilt);
  out.pitch = nf::clampf(std::atan2(df, dz), -c_.max_tilt, c_.max_tilt);
  out.thrust_n = c_.hover_n * gz / kG;
  return out;
}

// ------------------------------------------------------------------ Autopilot
const char* mode_name(Mode m) {
  switch (m) {
    case Mode::ANGLE: return "ANGLE";
    case Mode::POS_HOLD: return "POS_HOLD";
    case Mode::ALT_HOLD: return "ALT_HOLD";
    case Mode::LAND: return "LAND";
    case Mode::AUTO: return "AUTO";
  }
  return "?";
}

void Autopilot::init(const AutopilotConfig& c, const PosHoldConfig& pc) {
  c_ = c;
  law_.init(pc);
  mode_ = Mode::ANGLE;
  low_t_ = 0.0f;
  auto_v_ = {};
  auto_yaw_ = 0.0f;
}

void Autopilot::request_land() {
  if (mode_ != Mode::ANGLE) mode_ = Mode::LAND;
}

void Autopilot::engage(Mode m, const EstimateInput& est) {
  if (mode_ == Mode::ANGLE || mode_ == Mode::LAND) law_.reset(est.p);  // between hold modes: bumpless
  if (m == Mode::AUTO) { auto_v_ = {}; auto_yaw_ = 0.0f; goto_active_ = false; }
  mode_ = m;
}

static Vec3 heading_to_world(const Vec3& h, float yaw) {
  const float c = std::cos(yaw), s = std::sin(yaw);
  return {c * h.x - s * h.y, s * h.x + c * h.y, h.z};
}

AutopilotOutput Autopilot::update(const EstimateInput& est, const PilotInput& pilot, const CompanionInput& comp, float dt) {
  AutopilotOutput out;
  const bool z_valid = est.height_age < c_.valid_age_s;
  const bool xy_valid = est.flow_age < c_.valid_age_s && z_valid;
  const bool airborne = z_valid && est.p.z >= c_.min_engage_height;

  // 1. pilot mode requests (button edges)
  if (pilot.req_angle) {
    if (pilot.throttle >= c_.angle_min_throttle) mode_ = Mode::ANGLE;
    else out.refused = true;
  }
  if (pilot.req_hold) {
    if (airborne) engage(xy_valid ? Mode::POS_HOLD : Mode::ALT_HOLD, est);
    else out.refused = true;
  }
  if (pilot.req_auto) {
    if (airborne && xy_valid && comp.cmd_age_s <= c_.cmd_stale_s) engage(Mode::AUTO, est);  // laptop must be live
    else out.refused = true;
  }
  // 2. companion commands (AUTO only)
  if (mode_ == Mode::AUTO && comp.cmd_new) {
    const link::Cmd& k = comp.cmd;
    last_seq_ = k.seq;
    switch (k.op) {
      case link::OP_VEL:
        auto_v_ = {nf::clampf(k.a * 0.01f, -c_.cmd_v_max[0], c_.cmd_v_max[0]),
                   nf::clampf(k.b * 0.01f, -c_.cmd_v_max[1], c_.cmd_v_max[1]),
                   nf::clampf(k.c * 0.01f, -c_.cmd_v_max[2], c_.cmd_v_max[2])};
        auto_yaw_ = nf::clampf(k.yaw * 0.001f, -c_.max_auto_yaw_rate, c_.max_auto_yaw_rate);
        goto_active_ = false;
        break;
      case link::OP_GOTO: {
        const float m = c_.max_goto_m;
        const Vec3 d{nf::clampf(k.a * 0.01f, -m, m), nf::clampf(k.b * 0.01f, -m, m), nf::clampf(k.c * 0.01f, -m, m)};
        auto_v_ = {};
        auto_yaw_ = 0.0f;
        goto_target_ = est.p + heading_to_world(d, est.yaw);
        goto_active_ = true;   // flown as velocity commands below; survives a quiet laptop
        break;
      }
      case link::OP_LAND:
        mode_ = Mode::LAND;
        break;
      default:  // OP_HOLD: stop here (also cancels a GOTO)
        auto_v_ = {};
        auto_yaw_ = 0.0f;
        goto_active_ = false;
        break;
    }
  }
  if (mode_ == Mode::AUTO) {
    if (comp.cmd_age_s > c_.cmd_lost_s) {
      mode_ = Mode::POS_HOLD;          // companion gone: the pilot has the sticks, hold position
    } else if (comp.cmd_age_s > c_.cmd_stale_s) {
      auto_v_ = {};
      auto_yaw_ = 0.0f;                // late commands: stop and hover in place
    }
  }
  // 3. controller link lost in a hold mode: land (in ANGLE the arming logic cuts the motors)
  if (!pilot.link_ok && mode_ != Mode::ANGLE) mode_ = Mode::LAND;
  // 4. estimator failsafes (same order as lite/autonomy.py Autopilot)
  if ((mode_ == Mode::POS_HOLD || mode_ == Mode::AUTO) && !xy_valid && z_valid) mode_ = Mode::ALT_HOLD;
  if (mode_ != Mode::ANGLE && !z_valid) mode_ = Mode::LAND;
  if (mode_ == Mode::ALT_HOLD && xy_valid && pilot.link_ok) mode_ = Mode::POS_HOLD;

  out.mode = mode_;
  if (z_valid) h_last_ = est.p.z;
  if (mode_ == Mode::ANGLE) {
    low_t_ = 0.0f;
    blind_t_ = 0.0f;
    return out;
  }
  // 5. velocity command and law
  Vec3 cmd_h{};
  float yaw_rate = 0.0f;
  Vec3 cmd_w_extra{};
  if (mode_ == Mode::AUTO) {
    cmd_h = auto_v_;
    yaw_rate = auto_yaw_;
    if (goto_active_) {  // P to the GOTO point, limited to the command speeds
      Vec3 v = (goto_target_ - est.p) * c_.goto_kp;
      const float nxy = std::sqrt(v.x * v.x + v.y * v.y);
      if (nxy > c_.cmd_v_max[0]) { v.x *= c_.cmd_v_max[0] / nxy; v.y *= c_.cmd_v_max[0] / nxy; }
      v.z = nf::clampf(v.z, -c_.cmd_v_max[2], c_.cmd_v_max[2]);
      cmd_w_extra = v;
    }
  } else if (pilot.link_ok) {
    cmd_h = {pilot.fwd * c_.cmd_v_max[0], pilot.left * c_.cmd_v_max[1], pilot.up * c_.cmd_v_max[2]};
    yaw_rate = pilot.yaw_rate;
    if (mode_ == Mode::LAND) cmd_h.z = 0.0f;  // the pilot may steer a landing, not stop it
  }
  const LawMode lm = mode_ == Mode::LAND ? LawMode::LAND : (mode_ == Mode::ALT_HOLD ? LawMode::ALT : LawMode::POS);
  out.tt = law_.step(est.p, est.v, est.yaw, heading_to_world(cmd_h, est.yaw) + cmd_w_extra, lm, est.flow_age, z_valid, dt);
  if (mode_ == Mode::ALT_HOLD && pilot.link_ok) {
    const float mt = law_.config().max_tilt;
    out.tt.roll = nf::clampf(pilot.tilt_roll, -mt, mt);
    out.tt.pitch = nf::clampf(pilot.tilt_pitch, -mt, mt);
  }
  out.use_law = true;
  out.yaw_rate = yaw_rate;
  // 6. landing detector: in LAND, the controller asking for well under hover thrust means
  //    the ground is holding the drone up
  const bool low = mode_ == Mode::LAND && out.tt.thrust_n < c_.land_thrust_frac * law_.config().hover_n;
  low_t_ = low ? low_t_ + dt : 0.0f;
  blind_t_ = (mode_ == Mode::LAND && !z_valid) ? blind_t_ + dt : 0.0f;
  const float blind_limit =
      std::fmax(h_last_, 0.0f) / law_.config().land_speed * c_.blind_land_factor + c_.blind_land_margin_s;
  out.disarm_landed = low_t_ >= c_.land_detect_s || blind_t_ >= blind_limit;
  return out;
}

}  // namespace lite

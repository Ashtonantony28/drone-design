#include "lite/flight_core.hpp"

#include <cmath>

namespace lite {

using nf::Quat;
using nf::Vec3;

static float shape(float x, const StickConfig& c) {
  float a = std::fabs(x);
  if (a <= c.deadband) return 0.0f;
  a = (a - c.deadband) / (1.0f - c.deadband);
  a = nf::clampf(a, 0.0f, 1.0f);
  a = (1.0f - c.expo) * a + c.expo * a * a * a;
  return x < 0 ? -a : a;
}

Setpoint map_sticks(const Sticks& s, const StickConfig& c) {
  Setpoint sp;
  sp.roll = shape(s.roll, c) * c.max_tilt_rad;
  sp.pitch = shape(s.pitch, c) * c.max_tilt_rad;
  sp.yaw_rate = -shape(s.yaw, c) * c.max_yaw_rate;  // stick right = clockwise from above = -z in FLU
  sp.thrust_n = nf::clampf(s.throttle, 0.0f, 1.0f) * 4.0f * nf::vehicle::kThrustMax;
  return sp;
}

nf::CtbrCommand angle_mode(const Quat& q, const Setpoint& sp, const AngleConfig& c) {
  const nf::Mat3 R = q.to_mat();                    // body -> world
  const Vec3 b3 = R.col(2);
  // desired body z: the stick tilt applied in the current heading
  const Quat q_sp = Quat::from_euler(0.0f, 0.0f, q.yaw()) * Quat::from_euler(sp.roll, sp.pitch, 0.0f);
  const Vec3 d3 = q_sp.rotate(Vec3{0.0f, 0.0f, 1.0f});
  const Vec3 cr = nf::cross(b3, d3);
  const float s = nf::norm(cr);
  const float ang = std::atan2(s, nf::dot(b3, d3));
  const Vec3 axis = s > 1e-6f ? cr / s : R.col(0);   // exactly inverted: roll about body x
  const Vec3 e_b = R.tmul(axis * ang);
  nf::CtbrCommand out;
  out.rate_sp = Vec3{nf::clampf(c.kp[0] * e_b.x, -c.rate_limit.x, c.rate_limit.x),
                     nf::clampf(c.kp[1] * e_b.y, -c.rate_limit.y, c.rate_limit.y),
                     nf::clampf(sp.yaw_rate, -c.rate_limit.z, c.rate_limit.z)};
  const float cos_t = nf::clampf(b3.z, 0.5f, 1.0f);
  const float f = b3.z > 0.0f ? sp.thrust_n / cos_t : 0.3f * sp.thrust_n;
  out.thrust_n = nf::clampf(f, 0.0f, c.thrust_max_total);
  return out;
}

// ------------------------------------------------------------------ arming
void Arming::disarm(uint32_t why) {
  state_ = State::DISARMED;
  last_disarm_ = why;
  idle_ = tilt_high_ = false;
}

State Arming::update(const ArmingInputs& in) {
  const Sticks& s = in.sticks;
  const bool link = s.connected && s.age_ms <= c_.link_timeout_ms;
  uint32_t f = 0;
  if (!link) f |= NO_CONTROLLER;
  if (s.throttle > c_.arm_max_throttle) f |= THROTTLE_UP;
  if (in.tilt > c_.arm_max_tilt) f |= NOT_LEVEL;
  if (!in.gyro_cal_done) f |= GYRO_CAL;
  if (in.vbat < c_.vbat_empty) f |= BATTERY_EMPTY;
  if (in.vbat < c_.vbat_low) f |= BATTERY_LOW;
  flags_ = f;

  const bool arm_edge = s.arm && !arm_prev_;
  arm_prev_ = s.arm;

  switch (state_) {
    case State::CALIBRATING:
      if (in.gyro_cal_done) state_ = State::DISARMED;
      break;
    case State::DISARMED: {
      const uint32_t refuse = NO_CONTROLLER | THROTTLE_UP | NOT_LEVEL | GYRO_CAL | BATTERY_EMPTY;
      if (arm_edge && !s.kill && !(f & refuse)) {
        state_ = State::ARMED;
        last_disarm_ = 0;
        tilt_high_ = false;
        idle_ = true;  // the idle timer starts at arming
        idle_since_ = in.now_ms;
      }
      break;
    }
    case State::ARMED:
      if (s.kill) { disarm(KILLED); break; }
      if (!link && !in.hold_mode) { disarm(LINK_LOST); break; }
      if (in.tilt > c_.crash_tilt) {
        if (!tilt_high_) { tilt_high_ = true; tilt_since_ = in.now_ms; }
        if (in.now_ms - tilt_since_ >= c_.crash_ms) { disarm(CRASH); break; }
      } else {
        tilt_high_ = false;
      }
      if (s.throttle <= c_.arm_max_throttle && !in.hold_mode) {
        if (!idle_) { idle_ = true; idle_since_ = in.now_ms; }
        if (in.now_ms - idle_since_ >= c_.idle_disarm_ms) { disarm(IDLE_TIMEOUT); break; }
      } else {
        idle_ = false;
      }
      break;
  }
  return state_;
}

// ------------------------------------------------------------------ the core
void FlightCore::init() {
  const float fs = static_cast<float>(nf::vehicle::kRateLoopHz);
  cal_.init(nf::GyroCalibrator::Config{}, fs);
  mahony_.init(nf::Mahony::Config{});
  mahony_.reset();
  gyro_lpf_.init(150.0f, fs);  // neuroflight-fc ControlLoopConfig::gyro_lpf_hz
  pid_.init(nf::RatePidConfig{}, fs);
  mixer_.init(nf::MixerParams{});
  arming_.init(arming_cfg);
  tick_ = 0;
  attitude_seeded_ = false;
  // v3 estimator: neuroflight-fc ControlLoopConfig defaults (accel_lpf_hz 40, mahony_ref_lpf_hz 5)
  kf_.init(nf::TranslationKf::Config{});
  kf_.reset();
  acc_lpf_.init(40.0f, fs);
  for (auto& f : acc_ref_) { f.init(5.0f, fs); f.reset(0.0f); }
  prev_kf_vel_ = {};
  flow_age_ = Age{};
  height_age_ = Age{};
  ap_.init();
  ap_out_ = AutopilotOutput{};
  armed_prev_ = false;
}

// neuroflight-fc ControlLoop::accel_ref: soft deadband on the KF acceleration estimate
Vec3 FlightCore::accel_ref() const {
  Vec3 a{acc_ref_[0].value(), acc_ref_[1].value(), acc_ref_[2].value()};
  const float db = 0.3f;
  const float ah = std::sqrt(a.x * a.x + a.y * a.y);
  const float sh = ah > db ? (ah - db) / ah : 0.0f;
  a.x *= sh;
  a.y *= sh;
  a.z = std::fabs(a.z) > db ? a.z - nf::signf(a.z) * db : 0.0f;
  return a;
}

CoreOutput FlightCore::step(const ImuSample& imu, const Sticks& sticks, float vbat, uint32_t now_ms, const AuxInput* aux) {
  CoreOutput out;
  // 1. gyro calibration runs whenever disarmed (refuses to finish while moving)
  if (arming_.state() != State::ARMED) {
    if (cal_.update(imu.gyro, imu.accel)) mahony_.set_bias(cal_.bias());
  }
  if (!attitude_seeded_) {
    mahony_.init_from_accel(imu.accel);
    attitude_seeded_ = true;
  }
  // 2. attitude (KF-referenced Mahony: neuroflight-fc ControlLoop, mahony_kf_ref)
  const Vec3 g = gyro_lpf_.apply(imu.gyro);
  const Vec3 acc = acc_lpf_.apply(imu.accel);
  const Vec3 ar = accel_ref();
  mahony_.update(g, imu.accel, dt_, Vec3{ar.x, ar.y, ar.z + nf::vehicle::kGravity});
  const Vec3 rate = mahony_.rates(g);
  const Quat q = mahony_.q();
  const float tilt = nf::tilt_angle(q);
  // 2b. translation KF (only fed when the companion board sends flow / range)
  const bool was_armed = arming_.state() == State::ARMED;
  kf_.predict(acc, q, dt_);
  if (aux && aux->sample_new) kf_.correct(aux->flow, aux->range, q, rate);
  flow_age_.update(kf_.flow_valid(), dt_);
  height_age_.update(kf_.height_valid(), dt_);
  {
    const Vec3 v = kf_.vel();
    Vec3 a{};
    if (was_armed && kf_.height_valid() && kf_.flow_valid() && nf::finite(v) && nf::finite(prev_kf_vel_))
      a = (v - prev_kf_vel_) / dt_;
    const float ah = std::sqrt(a.x * a.x + a.y * a.y), amax = 6.0f;
    if (ah > amax) a = {a.x * amax / ah, a.y * amax / ah, a.z};
    a.z = nf::clampf(a.z, -0.5f * nf::vehicle::kGravity, 0.5f * nf::vehicle::kGravity);
    if (!nf::finite(a)) a = {};
    for (int i = 0; i < 3; ++i) acc_ref_[i].apply(a[i]);
    prev_kf_vel_ = v;
  }

  // 3. arming
  ArmingInputs ai;
  ai.gyro_cal_done = cal_.done();
  ai.tilt = tilt;
  ai.vbat = vbat;
  ai.sticks = sticks;
  ai.now_ms = now_ms;
  ai.hold_mode = was_armed && ap_.hold_mode();
  State st = arming_.update(ai);
  if (st == State::ARMED && !armed_prev_) {
    // arming: x/y start from here, height continues from the ToF
    const Vec3 p = kf_.pos();
    kf_.reset(Vec3{0.0f, 0.0f, p.z}, Vec3{0.0f, 0.0f, 0.0f});
    flow_age_ = Age{};
    height_age_ = Age{};
    prev_kf_vel_ = {};
    ap_.disarmed();
  }
  out.state = st;
  out.flags = arming_.flags() | arming_.last_disarm();
  out.tilt = tilt;
  out.q = q;
  out.pos = kf_.pos();
  out.vel = kf_.vel();
  out.height_valid = kf_.height_valid();
  out.flow_valid = kf_.flow_valid();

  // companion command: latch until the next outer-loop step
  if (aux) {
    if (aux->comp.cmd_new) { comp_.cmd = aux->comp.cmd; comp_.cmd_new = true; }
    comp_.cmd_age_s = aux->comp.cmd_age_s;
  } else {
    comp_.cmd_age_s = 1e3f;
  }
  // mode buttons: edges, released once after boot
  const bool btn[3] = {sticks.btn_hold, sticks.btn_angle, sticks.btn_auto};
  for (int i = 0; i < 3; ++i) {
    if (btn[i] && !btn_prev_[i]) btn_edge_[i] = true;  // latched until the next outer-loop step
    btn_prev_[i] = btn[i];
  }

  if (st != State::ARMED) {
    armed_prev_ = false;
    ap_.disarmed();
    ap_out_ = AutopilotOutput{};
    comp_.cmd_new = false;
    btn_edge_[0] = btn_edge_[1] = btn_edge_[2] = false;
    out.mode = Mode::ANGLE;
    pid_.reset();
    tick_ = 0;
    return out;  // motors 0
  }
  armed_prev_ = true;
  // 4. outer loop at kControlRateHz: Autopilot (hold modes) or angle mode from the sticks
  const bool outer = tick_ % static_cast<uint32_t>(outer_div_) == 0;
  if (outer) {
    const Setpoint sp = map_sticks(sticks, sticks_cfg);
    PilotInput pi;
    pi.link_ok = sticks.connected && sticks.age_ms <= arming_cfg.link_timeout_ms;
    pi.req_hold = btn_edge_[0];
    pi.req_angle = btn_edge_[1];
    pi.req_auto = btn_edge_[2];
    btn_edge_[0] = btn_edge_[1] = btn_edge_[2] = false;
    pi.throttle = sticks.throttle;
    pi.fwd = shape(sticks.pitch, sticks_cfg);   // stick forward = fly forward
    pi.left = -shape(sticks.roll, sticks_cfg);  // stick right = fly right
    pi.up = shape(sticks.climb, sticks_cfg);
    pi.tilt_roll = sp.roll;
    pi.tilt_pitch = sp.pitch;
    pi.yaw_rate = sp.yaw_rate;
    EstimateInput est;
    est.p = kf_.pos();
    est.v = kf_.vel();
    est.yaw = q.yaw();
    est.flow_age = flow_age_.age;
    est.height_age = height_age_.age;
    ap_out_ = ap_.update(est, pi, comp_, dt_ * static_cast<float>(outer_div_));
    comp_.cmd_new = false;
    out.refused = ap_out_.refused;
    if (ap_out_.disarm_landed) {
      arming_.force_disarm(LANDED);
      ap_.disarmed();
      out.state = arming_.state();
      out.flags = arming_.flags() | arming_.last_disarm();
      out.mode = Mode::ANGLE;
      pid_.reset();
      tick_ = 0;
      armed_prev_ = false;
      return out;
    }
    if (ap_out_.use_law) {
      Setpoint lp;
      lp.roll = ap_out_.tt.roll;
      lp.pitch = ap_out_.tt.pitch;
      lp.yaw_rate = ap_out_.yaw_rate;
      lp.thrust_n = ap_out_.tt.thrust_n;
      cmd_ = angle_mode(q, lp, angle_cfg);
    } else {
      cmd_ = angle_mode(q, sp, angle_cfg);
    }
  }
  out.mode = ap_.mode();
  ++tick_;
  // 5. rate PID + mixer (same order as sim/ctbr.py RateController)
  const Vec3 tau = pid_.update(cmd_.rate_sp, rate);
  const nf::MixerResult m = mixer_.mix(cmd_.thrust_n, tau);
  pid_.integrate(m.any_saturated());
  for (int i = 0; i < 4; ++i) out.motor[i] = nf::clampf(m.cmd[i], 0.0f, 1.0f);
  out.saturated = m.any_saturated();
  // zero throttle stops the motors in angle mode (no idle spin on a brushed micro)
  if (!ap_out_.use_law && sticks.throttle <= 0.0f)
    for (float& d : out.motor) d = 0.0f;
  return out;
}

}  // namespace lite

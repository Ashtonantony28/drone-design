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
      if (!link) { disarm(LINK_LOST); break; }
      if (in.tilt > c_.crash_tilt) {
        if (!tilt_high_) { tilt_high_ = true; tilt_since_ = in.now_ms; }
        if (in.now_ms - tilt_since_ >= c_.crash_ms) { disarm(CRASH); break; }
      } else {
        tilt_high_ = false;
      }
      if (s.throttle <= c_.arm_max_throttle) {
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
  arming_.init();
  tick_ = 0;
  attitude_seeded_ = false;
}

CoreOutput FlightCore::step(const ImuSample& imu, const Sticks& sticks, float vbat, uint32_t now_ms) {
  CoreOutput out;
  // 1. gyro calibration runs whenever disarmed (refuses to finish while moving)
  if (arming_.state() != State::ARMED) {
    if (cal_.update(imu.gyro, imu.accel)) mahony_.set_bias(cal_.bias());
  }
  if (!attitude_seeded_) {
    mahony_.init_from_accel(imu.accel);
    attitude_seeded_ = true;
  }
  // 2. attitude
  const Vec3 g = gyro_lpf_.apply(imu.gyro);
  mahony_.update(g, imu.accel, dt_);
  const Vec3 rate = mahony_.rates(g);
  const Quat q = mahony_.q();
  const float tilt = nf::tilt_angle(q);

  // 3. arming
  ArmingInputs ai;
  ai.gyro_cal_done = cal_.done();
  ai.tilt = tilt;
  ai.vbat = vbat;
  ai.sticks = sticks;
  ai.now_ms = now_ms;
  const State st = arming_.update(ai);
  out.state = st;
  out.flags = arming_.flags() | arming_.last_disarm();
  out.tilt = tilt;
  out.q = q;

  if (st != State::ARMED) {
    pid_.reset();
    tick_ = 0;
    return out;  // motors 0
  }
  // 4. outer loop at kControlRateHz
  if (tick_ % static_cast<uint32_t>(outer_div_) == 0) cmd_ = angle_mode(q, map_sticks(sticks, sticks_cfg), angle_cfg);
  ++tick_;
  // 5. rate PID + mixer (same order as sim/ctbr.py RateController)
  const Vec3 tau = pid_.update(cmd_.rate_sp, rate);
  const nf::MixerResult m = mixer_.mix(cmd_.thrust_n, tau);
  pid_.integrate(m.any_saturated());
  for (int i = 0; i < 4; ++i) out.motor[i] = nf::clampf(m.cmd[i], 0.0f, 1.0f);
  out.saturated = m.any_saturated();
  // zero throttle stops the motors (no idle spin on a brushed micro)
  if (sticks.throttle <= 0.0f)
    for (float& d : out.motor) d = 0.0f;
  return out;
}

}  // namespace lite

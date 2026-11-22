// Shared host-test rig: a tiny rigid-body quad sim on the Lite's vehicle params, plus the
// v3 flow / ToF sensors the companion board would send. Test code only.
#pragma once
#include <cmath>
#include <cstdio>

#include "lite/companion.hpp"
#include "lite/flight_core.hpp"

using namespace lite;
using nf::Quat;
using nf::Vec3;

// ---------------------------------------------------------------- tiny rigid-body sim
// Same physics as neuroflight-fc fw/core/tests/quad_sim.hpp, on the Lite's vehicle params.
struct Sim {
  Vec3 p{0, 0, 0}, v{}, omega{};
  Quat q{};
  float w[4] = {0, 0, 0, 0};
  Vec3 acc_w{};
  Vec3 ext_torque{};
  Vec3 ext_force{};   // world frame (N): wind gusts
  bool ground = true;
  float thrust(float wi) const {
    const float a = nf::vehicle::kThrustCurveA;
    return nf::vehicle::kThrustMax * (a * wi * wi + (1 - a) * wi);
  }
  void step(const float cmd[4], float dt) {
    using namespace nf::vehicle;
    const int sub = 4;
    const float h = dt / sub;
    for (int s = 0; s < sub; ++s) {
      float T[4];
      for (int i = 0; i < 4; ++i) {
        const float c = nf::clampf(cmd[i], 0.0f, 1.0f);
        const float tau = c > w[i] ? kMotorTauUp : kMotorTauDown;
        w[i] += (c - w[i]) * (h / tau);
        T[i] = thrust(w[i]);
      }
      const float a = kArmXy, k = kYawTorqueCoeff;
      Vec3 tau{a * (T[0] - T[1] - T[2] + T[3]), a * (-T[0] - T[1] + T[2] + T[3]), k * (T[0] - T[1] + T[2] - T[3])};
      tau += ext_torque;
      const Vec3 J{kInertia[0], kInertia[1], kInertia[2]};
      Vec3 wd = tau - nf::cross(omega, nf::hadamard(J, omega));
      wd = {wd.x / J.x, wd.y / J.y, wd.z / J.z};
      omega += wd * h;
      q = q.integrated(omega, h);
      const float F = T[0] + T[1] + T[2] + T[3];
      const Vec3 vb = q.rotate_inv(v);
      const Vec3 fb = Vec3{0, 0, F} - nf::hadamard(Vec3{kLinDrag[0], kLinDrag[1], kLinDrag[2]}, vb);
      Vec3 acc = (q.rotate(fb) + ext_force) / kMass + Vec3{0, 0, -kGravity};
      v += acc * h;
      p += v * h;
      if (ground && p.z <= 0.0f) {
        p.z = 0.0f;
        if (v.z < 0) v.z = 0;
        v.x *= 0.9f; v.y *= 0.9f;
        omega = omega * 0.9f;
        acc = {0, 0, 0};
      }
      acc_w = acc;
    }
  }
  ImuSample imu() const {
    ImuSample s;
    s.gyro = omega + Vec3{0.01f, -0.008f, 0.005f};  // constant bias: the calibrator must remove it
    s.accel = q.rotate_inv(acc_w + Vec3{0, 0, nf::vehicle::kGravity});
    return s;
  }
};

struct Rig {
  FlightCore fc;
  Sim sim;
  Sticks st;
  uint32_t ms = 0;
  CoreOutput out;
  // v3 companion sensors (off = the v2 board: no flow / ToF)
  bool sensors = false;
  bool flow_ok = true, range_ok = true;   // false = that sensor stops (unplugged / out of range)
  float flow_noise = 0.0f, range_noise = 0.0f;
  uint32_t rng = 12345u;
  AuxInput aux;
  float cmd_age = 1e3f;
  // wire = true: the sensors go through the real path: PMW3901 integer counts (fraction carried,
  // as the sensor accumulates), VL53L1X mm, companion::make_sens, SENS frame, link::Parser
  bool wire = false;
  float carry_a = 0.0f, carry_b = 0.0f;
  uint16_t seq = 0;
  link::Parser parser;
  Rig() { fc.init(); st.connected = true; }
  float noise() {  // uniform [-1, 1], deterministic
    rng = rng * 1664525u + 1013904223u;
    return static_cast<float>((rng >> 8) & 0xFFFF) / 32767.5f - 1.0f;
  }
  void send(const link::Cmd& c) { aux.comp.cmd = c; aux.comp.cmd_new = true; cmd_age = 0.0f; }
  void sense() {
    const nf::Mat3 R = sim.q.to_mat();
    const float r22 = R(2, 2);
    const float h = sim.p.z;
    aux.sample_new = true;
    aux.range.valid = range_ok && r22 > 0.3f && h / r22 < 4.0f;
    aux.range.range_m = h / r22 + range_noise * noise();
    const Vec3 vb = R.tmul(sim.v);
    const float hb = std::fmax(h / std::fmax(r22, 0.3f), 0.01f);
    aux.flow.valid = flow_ok && h > 0.08f;
    aux.flow.quality = 1.0f;
    aux.flow.flow_x = vb.x / hb - sim.omega.y + flow_noise * noise();
    aux.flow.flow_y = vb.y / hb + sim.omega.x + flow_noise * noise();
    if (wire) through_wire();
  }
  void through_wire() {
    const companion::FlowConfig fc;
    const float dt = 0.01f;
    // invert make_sens' axis mapping: body flow -> sensor counts
    const float a = aux.flow.flow_x * dt / fc.rad_per_count / fc.sign_x + carry_a;
    const float b = aux.flow.flow_y * dt / fc.rad_per_count / fc.sign_y + carry_b;
    const float ia = std::nearbyint(a), ib = std::nearbyint(b);
    carry_a = a - ia;
    carry_b = b - ib;
    const int16_t dx = static_cast<int16_t>(fc.swap_xy ? ib : ia), dy = static_cast<int16_t>(fc.swap_xy ? ia : ib);
    const uint16_t mm = static_cast<uint16_t>(nf::clampf(aux.range.range_m * 1000.0f, 0.0f, 65000.0f) + 0.5f);
    const link::Sens s = companion::make_sens(dx, dy, aux.flow.valid ? 80 : 0, dt, aux.flow.valid, mm, aux.range.valid,
                                              aux.range.valid ? 0 : 4, seq++);
    uint8_t buf[32];
    const size_t n = link::encode_sens(s, buf);
    aux.sample_new = false;
    for (size_t i = 0; i < n; ++i) {
      if (!parser.push(buf[i])) continue;
      link::Sens r;
      if (!link::decode_sens(parser.payload(), parser.len(), r)) continue;
      // the flight sketch's companion_poll() conversion
      aux.sample_new = true;
      aux.flow.flow_x = r.flow_x_mrad * 0.001f;
      aux.flow.flow_y = r.flow_y_mrad * 0.001f;
      aux.flow.quality = r.quality / 255.0f;
      aux.flow.valid = (r.flags & link::FLOW_OK) && aux.flow.valid;
      aux.range.range_m = r.range_mm * 0.001f;
      aux.range.valid = (r.flags & link::RANGE_OK) != 0;
    }
  }
  // hold = the drone is held still in the hand (calibration before an in-air test)
  void run(float seconds, float vbat = 3.9f, bool hold = false) {
    const int n = static_cast<int>(seconds * nf::vehicle::kRateLoopHz);
    for (int i = 0; i < n; ++i) {
      if (sensors) {
        if (ms % 10 == 0) sense();   // 100 Hz flow / ToF frames
        aux.comp.cmd_age_s = cmd_age;
      }
      out = fc.step(sim.imu(), st, vbat, ms, sensors ? &aux : nullptr);
      aux.sample_new = false;
      aux.comp.cmd_new = false;   // FlightCore latches it for the next outer-loop step
      cmd_age += 0.001f;
      if (!hold) sim.step(out.motor, 1.0f / nf::vehicle::kRateLoopHz);
      ms += 1;
    }
  }
  void arm(bool hold = false) {
    st.arm = true; run(0.01f, 3.9f, hold); st.arm = false; run(0.01f, 3.9f, hold);
  }
  void calibrate_in_hand() { run(1.5f, 3.9f, true); arm(true); }
};

static inline float deg(float r) { return r * 57.29578f; }

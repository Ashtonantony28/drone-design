// Host tests for the NF-3D Lite flight core (lite_fc + nf_core), no hardware.
//   lite/firmware/tests/run.sh
#include <cmath>
#include <cstdio>

#include "angle_fixture.hpp"
#include "lite/flight_core.hpp"
#include "lite_sim.hpp"
#include "nftest.hpp"

using namespace lite;
using nf::Quat;
using nf::Vec3;

#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))


// ---------------------------------------------------------------- unit tests
TEST(sticks_mapping) {
  Sticks s;
  s.roll = 0.03f; s.pitch = -0.05f;  // inside the deadband
  Setpoint sp = map_sticks(s);
  NEAR(sp.roll, 0.0f, 1e-6f); NEAR(sp.pitch, 0.0f, 1e-6f);
  s.roll = 1.0f; s.pitch = -1.0f; s.yaw = 1.0f; s.throttle = 1.0f;
  sp = map_sticks(s);
  NEAR(sp.roll, StickConfig{}.max_tilt_rad, 1e-5f);
  NEAR(sp.pitch, -StickConfig{}.max_tilt_rad, 1e-5f);
  CHECK(sp.yaw_rate < 0.0f);  // stick right = clockwise from above = -z (FLU)
  NEAR(sp.thrust_n, 4.0f * nf::vehicle::kThrustMax, 1e-5f);
  s.roll = 0.5f;
  CHECK(map_sticks(s).roll < 0.5f * StickConfig{}.max_tilt_rad);  // expo softens the centre
}

TEST(angle_mode_matches_simulated_law) {
  // lite/fly.py AngleMode, level setpoint, pilot at the nominal hover thrust
  Setpoint sp;
  sp.thrust_n = kFixtureHoverThrust;
  float worst = 0;
  for (int i = 0; i < kAngleCases; ++i) {
    const AngleCase& c = kAngle[i];
    const Quat q{c.q[0], c.q[1], c.q[2], c.q[3]};
    const nf::CtbrCommand cmd = angle_mode(q, sp);
    const float e = std::fmax(std::fmax(std::fabs(cmd.rate_sp.x - c.rate[0]), std::fabs(cmd.rate_sp.y - c.rate[1])),
                              std::fmax(std::fabs(cmd.rate_sp.z - c.rate[2]), std::fabs(cmd.thrust_n - c.thrust)));
    worst = std::fmax(worst, e);
  }
  std::printf("  angle_mode vs lite/fly.py: max |diff| %.2e over %d attitudes\n", worst, kAngleCases);
  CHECK(worst < 1e-4f);
}

TEST(angle_mode_signs) {
  Setpoint sp;
  sp.thrust_n = 0.5f;
  sp.roll = 0.3f;  // roll right from level -> positive roll rate
  CHECK(angle_mode(Quat{}, sp).rate_sp.x > 0.0f);
  sp.roll = 0; sp.pitch = 0.3f;  // nose down -> positive pitch rate
  CHECK(angle_mode(Quat{}, sp).rate_sp.y > 0.0f);
  // the stick tilt is applied in the current heading: yawed 90 deg left, a pitch
  // command is still a body pitch command
  const Quat yawed = Quat::from_euler(0, 0, 1.5708f);
  const nf::CtbrCommand c = angle_mode(yawed, sp);
  CHECK(c.rate_sp.y > 0.0f);
  NEAR(c.rate_sp.x, 0.0f, 1e-3f);
}

TEST(arming_rules) {
  Arming a;
  a.init();
  ArmingInputs in;
  in.sticks.connected = true;
  in.gyro_cal_done = false;
  in.sticks.arm = false;
  CHECK(a.update(in) == State::CALIBRATING);
  in.gyro_cal_done = true;
  CHECK(a.update(in) == State::DISARMED);
  in.sticks.throttle = 0.3f;  // throttle up: refuse
  in.sticks.arm = true; a.update(in); in.sticks.arm = false; a.update(in);
  CHECK(a.state() == State::DISARMED);
  CHECK(a.flags() & THROTTLE_UP);
  in.sticks.throttle = 0.0f;
  in.vbat = 3.2f;  // empty battery: refuse
  in.sticks.arm = true; a.update(in); in.sticks.arm = false; a.update(in);
  CHECK(a.state() == State::DISARMED);
  in.vbat = 3.9f;
  in.tilt = 0.6f;  // not level: refuse
  in.sticks.arm = true; a.update(in); in.sticks.arm = false; a.update(in);
  CHECK(a.state() == State::DISARMED);
  in.tilt = 0.0f;
  in.sticks.arm = true; a.update(in);
  CHECK(a.state() == State::ARMED);
  // holding arm does not re-trigger; kill disarms
  in.sticks.kill = true; a.update(in);
  CHECK(a.state() == State::DISARMED);
  CHECK(a.last_disarm() == KILLED);
  in.sticks.kill = false; in.sticks.arm = false; a.update(in);
  // link loss
  in.sticks.arm = true; a.update(in); in.sticks.arm = false;
  CHECK(a.state() == State::ARMED);
  in.sticks.throttle = 0.5f;
  in.sticks.age_ms = 300; a.update(in);
  CHECK(a.state() == State::DISARMED);
  CHECK(a.last_disarm() == LINK_LOST);
  in.sticks.age_ms = 0; in.sticks.throttle = 0.0f;
  // crash: tilted past 75 deg for 300 ms
  in.sticks.arm = true; a.update(in); in.sticks.arm = false;
  in.sticks.throttle = 0.5f;
  in.tilt = 1.5f;
  for (uint32_t t = 0; t <= 290; t += 10) { in.now_ms = 1000 + t; a.update(in); }
  CHECK(a.state() == State::ARMED);
  in.now_ms = 1310; a.update(in);
  CHECK(a.state() == State::DISARMED);
  CHECK(a.last_disarm() == CRASH);
  in.tilt = 0.0f; in.sticks.throttle = 0.0f;
  // idle timeout: armed at zero throttle for 5 s
  in.sticks.arm = true; in.now_ms = 2000; a.update(in); in.sticks.arm = false;
  in.now_ms = 6999; a.update(in);
  CHECK(a.state() == State::ARMED);
  in.now_ms = 7001; a.update(in);
  CHECK(a.state() == State::DISARMED);
  CHECK(a.last_disarm() == IDLE_TIMEOUT);
}

TEST(arm_must_be_released_after_boot) {
  Arming a;
  a.init();
  ArmingInputs in;
  in.sticks.connected = true;
  in.gyro_cal_done = true;
  in.sticks.arm = true;  // held at power-up (e.g. button stuck): no arm
  for (int i = 0; i < 5; ++i) a.update(in);
  CHECK(a.state() == State::DISARMED);
}

// ---------------------------------------------------------------- closed loop
TEST(closed_loop_takeoff_and_hover) {
  Rig r;
  r.run(1.5f);  // sit still: gyro calibration
  CHECK(r.out.state == State::DISARMED);
  CHECK(r.fc.mahony().bias().x > 0.005f);  // picked up the injected bias
  r.arm();
  CHECK(r.out.state == State::ARMED);
  // hover throttle from the propulsion model is ~0.59; 0.8 climbs
  r.st.throttle = 0.8f;
  r.run(1.0f);
  CHECK(r.sim.p.z > 0.3f);
  r.st.throttle = 0.59f;
  r.run(2.0f);
  std::printf("  hover: z %.2f m, tilt %.1f deg, xy drift %.2f m\n", r.sim.p.z, deg(nf::tilt_angle(r.sim.q)),
              std::sqrt(r.sim.p.x * r.sim.p.x + r.sim.p.y * r.sim.p.y));
  CHECK(r.out.state == State::ARMED);
  CHECK(deg(nf::tilt_angle(r.sim.q)) < 3.0f);
}

TEST(closed_loop_disturbance_rejection) {
  Rig r;
  r.sim.ground = false;
  r.sim.p.z = 2.0f;
  r.calibrate_in_hand();
  CHECK(r.out.state == State::ARMED);
  r.st.throttle = 0.59f;
  r.run(0.5f);
  r.sim.ext_torque = Vec3{1.2e-2f, -8e-3f, 0};  // a hard knock for 50 ms (about 1.5x the roll authority)
  r.run(0.05f);
  r.sim.ext_torque = Vec3{};
  const float peak = deg(nf::tilt_angle(r.sim.q));
  r.run(0.6f);
  const float after = deg(nf::tilt_angle(r.sim.q));
  std::printf("  knock: tilt %.1f deg at release, %.1f deg 0.6 s later\n", peak, after);
  CHECK(peak > 10.0f);
  CHECK(after < 5.0f);
}

TEST(closed_loop_stick_directions) {
  Rig r;
  r.sim.ground = false;
  r.sim.p.z = 2.0f;
  r.calibrate_in_hand();
  CHECK(r.out.state == State::ARMED);
  r.st.throttle = 0.59f;
  r.run(0.3f);
  r.st.roll = 1.0f;  // full right
  r.run(0.5f);
  const float roll = r.sim.q.roll();
  std::printf("  full right stick: roll %.1f deg (setpoint %.1f), vy %.2f m/s\n", deg(roll),
              deg(StickConfig{}.max_tilt_rad), r.sim.v.y);
  CHECK(roll > 0.4f);       // right side down
  CHECK(r.sim.v.y < -0.3f); // moving right (-y)
  r.st.roll = 0.0f;
  r.st.yaw = 1.0f;          // yaw right
  const float yaw0 = r.sim.q.yaw();
  r.run(0.5f);
  CHECK(nf::wrap_pi(r.sim.q.yaw() - yaw0) < -0.3f);  // clockwise from above
}

TEST(closed_loop_link_loss_stops_motors) {
  Rig r;
  r.sim.ground = false;
  r.sim.p.z = 2.0f;
  r.calibrate_in_hand();
  CHECK(r.out.state == State::ARMED);
  r.st.throttle = 0.59f;
  r.run(0.3f);
  r.st.age_ms = 400;  // controller went quiet
  r.run(0.01f);
  CHECK(r.out.state == State::DISARMED);
  for (float d : r.out.motor) CHECK(d == 0.0f);
}

int main() { return nftest::run_all(); }

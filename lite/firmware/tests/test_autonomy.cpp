// Host tests for the v3 autonomy layer: PositionHold parity with the simulated law, the
// companion link protocol, the Autopilot mode logic, and closed-loop flights with
// simulated flow / ToF sensors.   lite/firmware/tests/run.sh
#include <cmath>
#include <cstdio>
#include <cstring>

#include "lite/autonomy.hpp"
#include "lite_sim.hpp"
#include "nftest.hpp"
#include "pos_fixture.hpp"

#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

// ---------------------------------------------------------------- PositionHold parity
TEST(pos_hold_matches_simulated_law) {
  float worst = 0.0f;
  int worst_i = -1;
  for (int s = 0; s < kPosSeqs; ++s) {
    PositionHold law;
    PosHoldConfig c;
    c.hover_n = kPosFixtureHover;
    law.init(c);
    law.reset(Vec3{kPosStart[s][0], kPosStart[s][1], kPosStart[s][2]});
    for (int k = 0; k < kPosSteps; ++k) {
      const PosCase& r = kPos[s * kPosSteps + k];
      const TiltThrust o = law.step(Vec3{r.p[0], r.p[1], r.p[2]}, Vec3{r.v[0], r.v[1], r.v[2]}, r.yaw,
                                    Vec3{r.cmd[0], r.cmd[1], r.cmd[2]}, static_cast<LawMode>(r.mode), r.flow_age,
                                    r.z_valid != 0, kPosFixtureDt);
      const float e = std::fmax(std::fmax(std::fabs(o.roll - r.out[0]), std::fabs(o.pitch - r.out[1])),
                                std::fabs(o.thrust_n - r.out[2]));
      if (e > worst) { worst = e; worst_i = s * kPosSteps + k; }
    }
  }
  std::printf("  PositionHold vs lite/autonomy.py: max |diff| %.2e over %d steps (worst step %d)\n", worst,
              kPosSeqs * kPosSteps, worst_i);
  CHECK(worst < 1e-4f);
}

TEST(pos_hold_signs) {
  PositionHold law;
  law.init();
  law.reset(Vec3{0, 0, 1});
  // drone 0.5 m behind (-x) the hold point, facing +x: pitch nose down (+) to go forward
  TiltThrust o = law.step(Vec3{-0.5f, 0, 1}, Vec3{}, 0.0f, Vec3{}, LawMode::POS, 0.0f, true, 0.002f);
  CHECK(o.pitch > 0.05f);
  NEAR(o.roll, 0.0f, 1e-4f);
  // hold point to the left (+y): roll left = negative roll
  law.reset(Vec3{0, 0, 1});
  o = law.step(Vec3{0, -0.5f, 1}, Vec3{}, 0.0f, Vec3{}, LawMode::POS, 0.0f, true, 0.002f);
  CHECK(o.roll < -0.05f);
  // same, but the drone faces +y (yaw 90 deg): the target is straight ahead -> pitch forward
  law.reset(Vec3{0, 0, 1});
  o = law.step(Vec3{0, -0.5f, 1}, Vec3{}, 1.5708f, Vec3{}, LawMode::POS, 0.0f, true, 0.002f);
  CHECK(o.pitch > 0.05f);
  NEAR(o.roll, 0.0f, 1e-3f);
  // below the hold point: more than hover thrust; tilt never exceeds the limit
  law.reset(Vec3{0, 0, 1});
  o = law.step(Vec3{-50, 0, 0.5f}, Vec3{}, 0.0f, Vec3{}, LawMode::POS, 0.0f, true, 0.002f);
  CHECK(o.thrust_n > law.config().hover_n);
  CHECK(o.pitch <= law.config().max_tilt + 1e-6f);
  // LAND: descends even when height is unknown
  law.reset(Vec3{0, 0, 1});
  o = law.step(Vec3{0, 0, 1}, Vec3{}, 0.0f, Vec3{}, LawMode::LAND, 9.0f, false, 0.002f);
  CHECK(o.thrust_n < law.config().hover_n);
}

// ---------------------------------------------------------------- companion link
TEST(link_crc_known_vector) {
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(link::crc16(s, sizeof s) == 0x29B1);  // CRC-16/CCITT-FALSE check value
}

TEST(link_roundtrip_all_types) {
  uint8_t buf[64];
  link::Parser p;
  link::Sens s;
  s.flow_x_mrad = -1234; s.flow_y_mrad = 32000; s.quality = 201; s.flags = link::FLOW_OK | link::RANGE_OK;
  s.range_mm = 3999; s.seq = 65535;
  size_t n = link::encode_sens(s, buf);
  CHECK(n == link::kSensLen + link::kOverhead);
  int got = 0;
  for (size_t i = 0; i < n; ++i) got += p.push(buf[i]);
  CHECK(got == 1 && p.type() == link::SENS);
  link::Sens s2;
  CHECK(link::decode_sens(p.payload(), p.len(), s2));
  CHECK(s2.flow_x_mrad == s.flow_x_mrad && s2.flow_y_mrad == s.flow_y_mrad && s2.quality == s.quality &&
        s2.flags == s.flags && s2.range_mm == s.range_mm && s2.seq == s.seq);

  link::Cmd c;
  c.op = link::OP_GOTO; c.a = 150; c.b = -75; c.c = 20; c.yaw = -300; c.seq = 42;
  n = link::encode_cmd(c, buf);
  got = 0;
  for (size_t i = 0; i < n; ++i) got += p.push(buf[i]);
  link::Cmd c2;
  CHECK(got == 1 && p.type() == link::CMD && link::decode_cmd(p.payload(), p.len(), c2));
  CHECK(c2.op == c.op && c2.a == c.a && c2.b == c.b && c2.c == c.c && c2.yaw == c.yaw && c2.seq == c.seq);

  link::Telem t;
  t.mode = 4; t.state = 2; t.flags = 0x0401; t.pos_cm[0] = -12; t.pos_cm[1] = 340; t.pos_cm[2] = 150;
  t.vel_cms[2] = -40; t.yaw_mrad = 3141; t.vbat_mv = 3912; t.cmd_seq = 42;
  n = link::encode_telem(t, buf);
  got = 0;
  for (size_t i = 0; i < n; ++i) got += p.push(buf[i]);
  link::Telem t2;
  CHECK(got == 1 && p.type() == link::TELEM && link::decode_telem(p.payload(), p.len(), t2));
  CHECK(std::memcmp(&t, &t2, sizeof t) == 0 || (t2.pos_cm[1] == 340 && t2.vbat_mv == 3912 && t2.cmd_seq == 42));
  // wrong length / bad op are refused
  CHECK(!link::decode_cmd(p.payload(), p.len(), c2));
  uint8_t bad[link::kCmdLen] = {9};
  CHECK(!link::decode_cmd(bad, sizeof bad, c2));
}

// The same three frames are asserted by lite/ground/tests/test_link.py against the Python twin.
static bool frame_is(const uint8_t* b, size_t n, const char* hex) {
  if (std::strlen(hex) != 2 * n) return false;
  for (size_t i = 0; i < n; ++i) {
    unsigned v = 0;
    std::sscanf(hex + 2 * i, "%2x", &v);
    if (b[i] != v) return false;
  }
  return true;
}

TEST(link_vectors_match_python) {
  uint8_t b[64];
  link::Sens s;
  s.flow_x_mrad = -1234; s.flow_y_mrad = 32000; s.quality = 201; s.flags = 3; s.range_mm = 3999; s.seq = 65535;
  CHECK(frame_is(b, link::encode_sens(s, b), "a55a010a2efb007dc9039f0fffffbec0"));
  link::Cmd c;
  c.op = 1; c.a = 150; c.b = -75; c.c = 20; c.yaw = -300; c.seq = 42;
  CHECK(frame_is(b, link::encode_cmd(c, b), "a55a020b019600b5ff1400d4fe2a00925e"));
  link::Telem t;
  t.mode = 4; t.state = 2; t.flags = 0x0401; t.pos_cm[0] = -12; t.pos_cm[1] = 340; t.pos_cm[2] = 150;
  t.vel_cms[2] = -40; t.yaw_mrad = 3141; t.vbat_mv = 3912; t.cmd_seq = 42;
  CHECK(frame_is(b, link::encode_telem(t, b), "a55a811604020104f4ff5401960000000000d8ff450c480f2a007d4b"));
}

TEST(link_parser_fuzz) {
  // 2000 valid SENS frames with random garbage between them, some frames corrupted:
  // every intact frame is received with its contents, no corrupted one is accepted.
  uint32_t rng = 99;
  auto rnd = [&]() { rng = rng * 1103515245u + 12345u; return static_cast<uint8_t>(rng >> 16); };
  link::Parser p;
  uint8_t buf[64];
  int sent_ok = 0, recv_ok = 0, mismatches = 0, corrupted = 0;
  uint16_t expect_seq = 0;
  bool expect_valid = false;
  for (int f = 0; f < 2000; ++f) {
    const int junk = rnd() % 12;
    for (int j = 0; j < junk; ++j) {
      const uint8_t b = rnd();
      if (p.push(b)) ++mismatches;  // a frame out of pure noise would be a false accept
    }
    link::Sens s;
    s.seq = static_cast<uint16_t>(f);
    s.flow_x_mrad = static_cast<int16_t>(rnd() * 37 - 4000);
    s.range_mm = static_cast<uint16_t>(rnd() * 13);
    const size_t n = link::encode_sens(s, buf);
    const bool corrupt = rnd() < 25;  // ~10 %
    if (corrupt) { buf[2 + rnd() % (n - 2)] ^= static_cast<uint8_t>(1u << (rnd() % 8)); ++corrupted; }
    else ++sent_ok;
    expect_seq = s.seq;
    expect_valid = !corrupt;
    for (size_t i = 0; i < n; ++i) {
      if (p.push(buf[i])) {
        link::Sens r;
        if (p.type() == link::SENS && link::decode_sens(p.payload(), p.len(), r) && expect_valid && r.seq == expect_seq &&
            r.flow_x_mrad == s.flow_x_mrad && r.range_mm == s.range_mm)
          ++recv_ok;
        else
          ++mismatches;
      }
    }
  }
  std::printf("  parser fuzz: %d intact frames sent, %d received, %d corrupted, %d false accepts; crc errors %u\n",
              sent_ok, recv_ok, corrupted, mismatches, p.crc_errors());
  CHECK(mismatches == 0);
  // a corrupted length byte can swallow the next frame's start: allow a handful of losses
  CHECK(recv_ok >= sent_ok - corrupted / 4);
  CHECK(recv_ok <= sent_ok);
}

// ---------------------------------------------------------------- companion helpers
TEST(companion_flow_conversion) {
  companion::FlowConfig fc;
  // 10 counts on the sensor's y axis in 10 ms -> body x (swap) with sign -1
  link::Sens s = companion::make_sens(0, 10, 80, 0.01f, true, 500, true, 0, 1, fc);
  CHECK(s.flags & link::FLOW_OK);
  NEAR(s.flow_x_mrad * 0.001f, -10 * fc.rad_per_count / 0.01f, 1e-3f);
  CHECK(s.flow_y_mrad == 0);
  CHECK(s.quality == 204);  // 80 / 100 * 255
  CHECK((s.flags & link::RANGE_OK) && s.range_mm == 500);
  // stale or bad range samples are never flagged valid
  s = companion::make_sens(0, 0, 80, 0.01f, true, 500, false, 0, 2, fc);
  CHECK(!(s.flags & link::RANGE_OK));
  s = companion::make_sens(0, 0, 80, 0.01f, true, 500, true, 4, 3, fc);   // status 4 = out of bounds
  CHECK(!(s.flags & link::RANGE_OK));
  s = companion::make_sens(0, 0, 80, 0.01f, true, 20, true, 0, 4, fc);    // below the minimum
  CHECK(!(s.flags & link::RANGE_OK));
  // a stalled loop (dt 0.5 s) or a lens-covered sensor gives no flow
  s = companion::make_sens(5, 5, 80, 0.5f, true, 500, true, 0, 5, fc);
  CHECK(!(s.flags & link::FLOW_OK));
  s = companion::make_sens(0, 0, 0, 0.01f, false, 500, true, 0, 6, fc);
  CHECK(s.quality == 0);
}

TEST(companion_bridge_forwards_only_commands) {
  uint8_t in[128], out[128];
  size_t n = 0;
  link::Cmd c; c.op = link::OP_VEL; c.a = 30; c.seq = 5;
  link::Telem t;
  n += link::encode_telem(t, in + n);       // wrong type: dropped
  in[n++] = 0x00; in[n++] = 0xA5;           // junk
  n += link::encode_cmd(c, in + n);         // forwarded
  const size_t bad = n;
  n += link::encode_cmd(c, in + n);
  in[bad + 6] ^= 0x10;                      // corrupted command: dropped
  link::Parser p;
  const size_t w = companion::bridge_from_laptop(in, n, out, sizeof out, p);
  CHECK(w == link::kCmdLen + link::kOverhead);
  link::Parser q;
  link::Cmd r;
  int frames = 0;
  for (size_t i = 0; i < w; ++i)
    if (q.push(out[i]) && link::decode_cmd(q.payload(), q.len(), r)) ++frames;
  CHECK(frames == 1 && r.a == 30 && r.seq == 5);
}

// ---------------------------------------------------------------- Autopilot unit tests
struct ApRig {
  Autopilot ap;
  EstimateInput est;
  PilotInput pilot;
  CompanionInput comp;
  AutopilotOutput out;
  ApRig() {
    ap.init();
    est.p = Vec3{0, 0, 1.0f};
    est.flow_age = est.height_age = 0.0f;
    pilot.link_ok = true;
    pilot.throttle = 0.6f;
    comp.cmd_age_s = 0.0f;
  }
  void tick(float secs = 0.002f) {
    const int n = static_cast<int>(secs / 0.002f + 0.5f);
    for (int i = 0; i < (n < 1 ? 1 : n); ++i) {
      out = ap.update(est, pilot, comp, 0.002f);
      pilot.req_hold = pilot.req_angle = pilot.req_auto = false;
      comp.cmd_new = false;
    }
  }
};

TEST(autopilot_engage_rules) {
  ApRig r;
  r.est.p.z = 0.05f;  // on the ground: hold refused
  r.pilot.req_hold = true; r.tick();
  CHECK(r.out.refused && r.ap.mode() == Mode::ANGLE);
  r.est.p.z = 1.0f;
  r.pilot.req_auto = true; r.tick();
  CHECK(r.ap.mode() == Mode::AUTO);
  r.pilot.req_hold = true; r.tick();
  CHECK(r.ap.mode() == Mode::POS_HOLD);
  // X with the trigger released would drop the drone: refused
  r.pilot.throttle = 0.0f; r.pilot.req_angle = true; r.tick();
  CHECK(r.out.refused && r.ap.mode() == Mode::POS_HOLD);
  r.pilot.throttle = 0.6f; r.pilot.req_angle = true; r.tick();
  CHECK(r.ap.mode() == Mode::ANGLE && !r.out.use_law);
  // no flow: A gives ALT_HOLD, Y is refused
  r.est.flow_age = 0.8f;
  r.pilot.req_auto = true; r.tick();
  CHECK(r.out.refused && r.ap.mode() == Mode::ANGLE);
  r.pilot.req_hold = true; r.tick();
  CHECK(r.ap.mode() == Mode::ALT_HOLD);
}

TEST(autopilot_failsafes) {
  ApRig r;
  r.pilot.req_hold = true; r.tick();
  CHECK(r.ap.mode() == Mode::POS_HOLD);
  // flow gap shorter than 0.5 s: stays in POS_HOLD (coasting on the KF)
  r.est.flow_age = 0.4f; r.tick();
  CHECK(r.ap.mode() == Mode::POS_HOLD);
  r.est.flow_age = 0.6f; r.tick();
  CHECK(r.ap.mode() == Mode::ALT_HOLD);
  r.est.flow_age = 0.0f; r.tick();
  CHECK(r.ap.mode() == Mode::POS_HOLD);
  // height lost: LAND, latched even when it comes back
  r.est.height_age = 0.6f; r.tick();
  CHECK(r.ap.mode() == Mode::LAND);
  r.est.height_age = 0.0f; r.tick();
  CHECK(r.ap.mode() == Mode::LAND);
  // the pilot can take it back with A once the sensors are fine
  r.pilot.req_hold = true; r.tick();
  CHECK(r.ap.mode() == Mode::POS_HOLD);
  // controller link lost in a hold mode: LAND
  r.pilot.link_ok = false; r.tick();
  CHECK(r.ap.mode() == Mode::LAND);
  r.pilot.link_ok = true; r.tick();
  CHECK(r.ap.mode() == Mode::LAND);
}

TEST(autopilot_companion_commands) {
  ApRig r;
  r.pilot.req_auto = true; r.tick();
  CHECK(r.ap.mode() == Mode::AUTO);
  // velocity command, clamped to cmd_v_max
  link::Cmd c;
  c.op = link::OP_VEL; c.a = 500; c.seq = 7;
  r.comp.cmd = c; r.comp.cmd_new = true; r.tick();
  CHECK(r.ap.last_cmd_seq() == 7);
  CHECK(r.out.tt.pitch > 0.0f);  // accelerating forward
  // GOTO 1 m forward while facing +y: flies towards +y = straight ahead (pitch forward, no roll)
  c.op = link::OP_HOLD; c.seq = 8;
  r.comp.cmd = c; r.comp.cmd_new = true; r.tick(0.5f);
  r.est.yaw = 1.5708f;
  c.op = link::OP_GOTO; c.a = 100; c.seq = 8;
  r.comp.cmd = c; r.comp.cmd_new = true; r.tick(0.1f);
  CHECK(r.out.tt.pitch > 0.02f);
  NEAR(r.out.tt.roll, 0.0f, 0.02f);
  // commands in POS_HOLD are ignored
  r.pilot.req_hold = true; r.tick();
  c.op = link::OP_LAND; c.seq = 9;
  r.comp.cmd = c; r.comp.cmd_new = true; r.tick();
  CHECK(r.ap.mode() == Mode::POS_HOLD && r.ap.last_cmd_seq() == 8);
  // stale commands: stop, then hand back to POS_HOLD
  r.pilot.req_auto = true; r.tick();
  c.op = link::OP_VEL; c.a = 50; c.seq = 10;
  r.comp.cmd = c; r.comp.cmd_new = true; r.tick();
  r.comp.cmd_age_s = 1.0f; r.tick();
  CHECK(r.ap.mode() == Mode::AUTO);
  r.comp.cmd_age_s = 2.5f; r.tick();
  CHECK(r.ap.mode() == Mode::POS_HOLD);
  // companion LAND
  r.comp.cmd_age_s = 0.0f;
  r.pilot.req_auto = true; r.tick();
  c.op = link::OP_LAND; c.seq = 11;
  r.comp.cmd = c; r.comp.cmd_new = true; r.tick();
  CHECK(r.ap.mode() == Mode::LAND);
}

// ---------------------------------------------------------------- closed loop with flow / ToF
// Take off in angle mode on the trigger, climb to ~1 m, press A.
static void takeoff_and_hold(Rig& r) {
  r.sensors = true;
  r.flow_noise = 0.05f;
  r.range_noise = 0.005f;
  r.run(1.5f);
  r.arm();
  r.st.throttle = 0.75f;
  for (int i = 0; i < 300 && r.sim.p.z < 0.9f; ++i) r.run(0.01f);
  r.st.throttle = 0.62f;
  r.st.btn_hold = true; r.run(0.01f); r.st.btn_hold = false;
  r.run(2.0f);  // settle on the hold point
}

static float xy_dist(const Vec3& a, const Vec3& b) { return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)); }

TEST(closed_loop_position_hold_with_gusts) {
  Rig r;
  takeoff_and_hold(r);
  CHECK(r.out.mode == Mode::POS_HOLD);
  r.st.throttle = 0.0f;  // trigger released: ignored in hold modes, and no idle disarm
  r.run(2.0f);
  const Vec3 p0 = r.sim.p;
  float worst = 0, worst_z = 0;
  for (int k = 0; k < 1000; ++k) {  // 10 s, 1.5 s gusts of 0.06 N (~8 % of the weight) sideways
    r.sim.ext_force = (k / 150) % 2 ? Vec3{0.05f, -0.03f, 0} : Vec3{};
    r.run(0.01f);
    worst = std::fmax(worst, xy_dist(r.sim.p, p0));
    worst_z = std::fmax(worst_z, std::fabs(r.sim.p.z - p0.z));
  }
  r.sim.ext_force = {};
  r.run(2.0f);
  const float final_err = xy_dist(r.sim.p, p0);
  std::printf("  hold at %.2f m with gusts: worst xy %.3f m, worst z %.3f m, after gusts %.3f m, armed %d\n", p0.z, worst,
              worst_z, final_err, r.out.state == State::ARMED);
  CHECK(r.out.state == State::ARMED && r.out.mode == Mode::POS_HOLD);
  CHECK(worst < 0.25f);
  CHECK(worst_z < 0.10f);
  CHECK(final_err < 0.08f);
}

TEST(closed_loop_hold_through_the_wire) {
  // same hold, but flow and range go through PMW3901 integer counts, the SENS frame and the parser
  Rig r;
  r.wire = true;
  takeoff_and_hold(r);
  CHECK(r.out.mode == Mode::POS_HOLD);
  const Vec3 p0 = r.sim.p;
  float worst = 0;
  for (int k = 0; k < 1000; ++k) {
    r.sim.ext_force = (k / 150) % 2 ? Vec3{-0.03f, 0.05f, 0} : Vec3{};
    r.run(0.01f);
    worst = std::fmax(worst, xy_dist(r.sim.p, p0));
  }
  std::printf("  hold through the wire (count quantisation, mm range): worst xy %.3f m\n", worst);
  CHECK(r.out.mode == Mode::POS_HOLD && r.out.state == State::ARMED);
  CHECK(worst < 0.25f);
}

TEST(closed_loop_velocity_sticks_and_stop) {
  Rig r;
  takeoff_and_hold(r);
  r.run(1.0f);
  const Vec3 p0 = r.sim.p;
  r.st.pitch = 1.0f;  // full forward: cmd_v_max forward (+x, drone faces +x)
  r.run(2.5f);
  const float vx = r.sim.v.x;
  r.st.pitch = 0.0f;
  r.run(0.5f);
  const Vec3 p_stop = r.sim.p;
  r.run(3.0f);
  std::printf("  forward stick: vx %.2f m/s (cmd %.2f), moved %.2f m, drift after stop %.3f m\n", vx,
              nf::vehicle::kCmdVelMax[0], r.sim.p.x - p0.x, xy_dist(r.sim.p, p_stop));
  NEAR(vx, nf::vehicle::kCmdVelMax[0], 0.15f);
  CHECK(xy_dist(r.sim.p, p_stop) < 0.25f);
  // climb on the left stick
  const float z0 = r.sim.p.z;
  r.st.climb = 1.0f;
  r.run(1.0f);
  r.st.climb = 0.0f;
  r.run(1.0f);
  CHECK(r.sim.p.z > z0 + 0.3f);
}

TEST(closed_loop_flow_loss_holds_height) {
  Rig r;
  takeoff_and_hold(r);
  r.run(1.0f);
  const float z0 = r.sim.p.z;
  r.flow_ok = false;
  float worst_z = 0;
  for (int k = 0; k < 500; ++k) { r.run(0.01f); worst_z = std::fmax(worst_z, std::fabs(r.sim.p.z - z0)); }
  std::printf("  flow lost: mode %s, height error %.3f m, tilt %.1f deg\n", mode_name(r.out.mode), worst_z,
              deg(nf::tilt_angle(r.sim.q)));
  CHECK(r.out.mode == Mode::ALT_HOLD);
  CHECK(worst_z < 0.15f);
  r.flow_ok = true;
  r.run(0.5f);
  CHECK(r.out.mode == Mode::POS_HOLD);
}

TEST(closed_loop_range_loss_lands_and_disarms) {
  Rig r;
  takeoff_and_hold(r);
  r.run(1.0f);
  r.range_ok = false;
  float td = -1;
  for (int k = 0; k < 1500 && r.out.state == State::ARMED; ++k) {
    const float vz = r.sim.v.z;
    r.run(0.01f);
    if (td < 0 && r.sim.p.z <= 0.0f) td = -vz;
  }
  std::printf("  range lost: touchdown %.2f m/s, state %d, last disarm 0x%x\n", td, static_cast<int>(r.out.state),
              r.out.flags & LANDED);
  CHECK(td >= 0.0f && td < 1.0f);
  CHECK(r.out.state == State::DISARMED);
  CHECK(r.out.flags & LANDED);
}

TEST(closed_loop_link_loss_in_hold_lands) {
  Rig r;
  takeoff_and_hold(r);
  r.run(1.0f);
  r.st.age_ms = 400;  // controller silent: in a hold mode the drone lands instead of falling
  float td = -1;
  for (int k = 0; k < 1500 && r.out.state == State::ARMED; ++k) {
    const float vz = r.sim.v.z;
    r.run(0.01f);
    if (td < 0 && r.sim.p.z <= 0.0f) td = -vz;
  }
  std::printf("  link lost in POS_HOLD: touchdown %.2f m/s, disarmed %d\n", td, r.out.state == State::DISARMED);
  CHECK(td >= 0.0f && td < 0.6f);
  CHECK(r.out.state == State::DISARMED && (r.out.flags & LANDED));
}

TEST(closed_loop_low_battery_in_hold_lands) {
  Rig r;
  takeoff_and_hold(r);
  r.run(1.0f);
  r.run(2.1f, 3.2f);  // under vbat_land for 2 s
  CHECK(r.out.mode == Mode::LAND && (r.out.flags & BATTERY_LAND));
  float td = -1;
  for (int k = 0; k < 1500 && r.out.state == State::ARMED; ++k) {
    const float vz = r.sim.v.z;
    r.run(0.01f, 3.2f);
    if (td < 0 && r.sim.p.z <= 0.0f) td = -vz;
  }
  std::printf("  low battery in POS_HOLD: touchdown %.2f m/s, disarmed %d\n", td, r.out.state == State::DISARMED);
  CHECK(td >= 0.0f && td < 0.6f);
  CHECK(r.out.state == State::DISARMED && (r.out.flags & LANDED));
}

TEST(closed_loop_auto_follow_goto_land) {
  Rig r;
  takeoff_and_hold(r);
  r.run(1.0f);
  r.st.btn_auto = true; r.run(0.01f); r.st.btn_auto = false;
  r.run(0.01f);
  CHECK(r.out.mode == Mode::POS_HOLD);  // no laptop yet: Y refused
  link::Cmd hold; hold.op = link::OP_HOLD;
  r.send(hold);
  r.st.btn_auto = true; r.run(0.01f); r.st.btn_auto = false;
  CHECK(r.out.mode == Mode::AUTO);
  const Vec3 p0 = r.sim.p;
  // laptop: 0.4 m/s forward at 15 Hz for 2 s
  link::Cmd c;
  c.op = link::OP_VEL; c.a = 40;
  for (int k = 0; k < 30; ++k) { c.seq = static_cast<uint16_t>(k); r.send(c); r.run(1.0f / 15); }
  const float vx = r.sim.v.x;
  // stop, then GOTO 0.5 m left; the GOTO completes even though the laptop then stays quiet
  for (int k = 0; k < 15; ++k) { r.send(hold); r.run(1.0f / 15); }
  const Vec3 p0b = r.sim.p;
  c.op = link::OP_GOTO; c.a = 0; c.b = 50; r.send(c); r.run(1.8f);
  const Vec3 p1 = r.sim.p;
  std::printf("  AUTO: vx %.2f m/s (cmd 0.40, moved %.2f m), goto 0.5 m left: dx %.2f dy %.2f m\n", vx, p0b.x - p0.x,
              p1.x - p0b.x, p1.y - p0b.y);
  NEAR(vx, 0.40f, 0.1f);
  NEAR(p1.y - p0b.y, 0.5f, 0.1f);
  NEAR(p1.x - p0b.x, 0.0f, 0.1f);
  // laptop goes silent: hover in place, then POS_HOLD
  r.run(3.0f);
  CHECK(r.out.mode == Mode::POS_HOLD);
  CHECK(xy_dist(r.sim.p, p1) < 0.15f);
  // back to AUTO, companion LAND
  r.st.btn_auto = true; r.send(hold); r.run(0.01f); r.st.btn_auto = false;
  CHECK(r.out.mode == Mode::AUTO);
  c.op = link::OP_LAND; r.send(c);
  float td = -1;
  for (int k = 0; k < 1000 && r.out.state == State::ARMED; ++k) {
    const float vz = r.sim.v.z;
    r.run(0.01f);
    if (td < 0 && r.sim.p.z <= 0.0f) td = -vz;
  }
  std::printf("  AUTO land: touchdown %.2f m/s, disarmed %d\n", td, r.out.state == State::DISARMED);
  CHECK(td >= 0.0f && td < 0.6f);
  CHECK(r.out.state == State::DISARMED);
}

TEST(closed_loop_kill_works_in_every_mode) {
  for (int m = 0; m < 3; ++m) {
    Rig r;
    takeoff_and_hold(r);
    r.run(0.5f);
    if (m == 1) { r.send(link::Cmd{}); r.st.btn_auto = true; r.run(0.01f); r.st.btn_auto = false; CHECK(r.out.mode == Mode::AUTO); }
    if (m == 2) { r.range_ok = false; r.run(0.7f); CHECK(r.out.mode == Mode::LAND); }
    r.st.kill = true;
    r.run(0.01f);
    CHECK(r.out.state == State::DISARMED && (r.out.flags & KILLED));
    for (float d : r.out.motor) CHECK(d == 0.0f);
  }
}

TEST(v2_board_without_sensors_is_unchanged) {
  // no companion: A / Y are refused (no height), the drone stays in angle mode
  Rig r;
  r.sim.ground = false;
  r.sim.p.z = 2.0f;
  r.calibrate_in_hand();
  r.st.throttle = 0.6f;
  r.run(0.3f);
  r.st.btn_hold = true; r.run(0.01f); r.st.btn_hold = false;
  CHECK(r.out.mode == Mode::ANGLE);
  r.st.btn_auto = true; r.run(0.01f); r.st.btn_auto = false;
  CHECK(r.out.mode == Mode::ANGLE);
  CHECK(r.out.state == State::ARMED);
}

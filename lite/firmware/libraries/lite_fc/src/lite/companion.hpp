// NF-3D Lite v3 companion (XIAO ESP32-S3 Sense) logic with no hardware calls, so it is
// host-tested: PMW3901 counts -> body flow (nf::FlowSample convention), VL53L1X sample
// gating, the SENS frame, and the laptop <-> flight-board bridge filter.
#pragma once

#include <cmath>
#include <cstdint>

#include "lite/link.hpp"

namespace lite {
namespace companion {

struct FlowConfig {
  // rad of image motion per PMW3901 count. Crazyflie flow-deck model: thetapix / Npix with
  // thetapix = 4.2 deg, Npix = 35. VERIFY with the slide test (BUILD.md "flow calibration").
  float rad_per_count = 4.2f * 0.017453293f / 35.0f;
  // sensor axes -> body (FLU) axes, set by how the module sits on the tail tab; the slide test
  // checks these too: slide forward -> flow_x > 0, slide left -> flow_y > 0
  bool swap_xy = true;
  float sign_x = -1.0f, sign_y = -1.0f;
  float squal_full = 100.0f;   // SQUAL giving quality 1.0 (nf KF needs >= 0.3, i.e. SQUAL >= 30)
};

struct RangeConfig {
  uint16_t min_mm = 40, max_mm = 4000;  // VL53L1X short mode is good to ~1.3 m, long to 4 m
};

// One sensor frame from the raw readings. dt_s: time since the previous flow read.
// range_new: a fresh VL53L1X sample (only fresh ones are flagged valid, so the KF never
// fuses the same range twice); range_status: VL53L1X RangeStatus (0 = valid).
inline link::Sens make_sens(int16_t dx, int16_t dy, uint8_t squal, float dt_s, bool flow_motion, uint16_t range_mm,
                            bool range_new, uint8_t range_status, uint16_t seq, const FlowConfig& fc = FlowConfig{},
                            const RangeConfig& rc = RangeConfig{}) {
  link::Sens s;
  s.seq = seq;
  float fx = 0.0f, fy = 0.0f;
  const bool dt_ok = dt_s > 0.002f && dt_s < 0.1f;
  if (dt_ok) {
    const float a = static_cast<float>(fc.swap_xy ? dy : dx), b = static_cast<float>(fc.swap_xy ? dx : dy);
    fx = fc.sign_x * a * fc.rad_per_count / dt_s;
    fy = fc.sign_y * b * fc.rad_per_count / dt_s;
  }
  const float lim = 32.0f;  // rad/s, int16 mrad range
  const bool sane = std::fabs(fx) < lim && std::fabs(fy) < lim;
  float q = squal / fc.squal_full;
  q = q < 0.0f ? 0.0f : (q > 1.0f ? 1.0f : q);
  if (!flow_motion && squal == 0) q = 0.0f;   // sensor not ready / lens covered
  s.quality = static_cast<uint8_t>(q * 255.0f + 0.5f);
  if (dt_ok && sane) {
    s.flow_x_mrad = static_cast<int16_t>(std::lround(fx * 1000.0f));
    s.flow_y_mrad = static_cast<int16_t>(std::lround(fy * 1000.0f));
    s.flags |= link::FLOW_OK;
  }
  s.range_mm = range_mm;
  if (range_new && range_status == 0 && range_mm >= rc.min_mm && range_mm <= rc.max_mm) s.flags |= link::RANGE_OK;
  return s;
}

// Laptop -> flight board: only well-formed CMD frames are forwarded (the bridge re-encodes them,
// so a corrupted UDP datagram can never reach the flight board's UART as anything else).
// Returns the number of bytes written to out (0 = drop).
inline size_t bridge_from_laptop(const uint8_t* data, size_t n, uint8_t* out, size_t out_cap, link::Parser& p) {
  size_t w = 0;
  for (size_t i = 0; i < n; ++i) {
    if (!p.push(data[i]) || p.type() != link::CMD) continue;
    link::Cmd c;
    if (!link::decode_cmd(p.payload(), p.len(), c)) continue;
    if (w + link::kCmdLen + link::kOverhead > out_cap) break;
    w += link::encode_cmd(c, out + w);
  }
  return w;
}

}  // namespace companion
}  // namespace lite

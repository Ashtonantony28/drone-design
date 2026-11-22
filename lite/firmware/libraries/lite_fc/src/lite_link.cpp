#include "lite/link.hpp"

#include <cstring>

namespace lite {

namespace link {

uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc) {
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint16_t>(d[i]) << 8;
    for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

size_t encode(uint8_t type, const uint8_t* payload, size_t len, uint8_t* out) {
  if (len > kMaxPayload) return 0;
  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = type;
  out[3] = static_cast<uint8_t>(len);
  if (len) std::memcpy(out + 4, payload, len);
  const uint16_t crc = crc16(out + 2, len + 2);
  out[4 + len] = static_cast<uint8_t>(crc & 0xFF);
  out[5 + len] = static_cast<uint8_t>(crc >> 8);
  return len + kOverhead;
}

namespace {
struct W {
  uint8_t* p;
  size_t i;  // no default initializer: aggregate init must work in C++11 (esp32_bluepad32 core)
  void u8(uint8_t v) { p[i++] = v; }
  void u16(uint16_t v) { p[i++] = static_cast<uint8_t>(v & 0xFF); p[i++] = static_cast<uint8_t>(v >> 8); }
  void i16(int16_t v) { u16(static_cast<uint16_t>(v)); }
};
struct R {
  const uint8_t* p;
  size_t i;  // no default initializer: aggregate init must work in C++11 (esp32_bluepad32 core)
  uint8_t u8() { return p[i++]; }
  uint16_t u16() { const uint16_t v = static_cast<uint16_t>(p[i] | (p[i + 1] << 8)); i += 2; return v; }
  int16_t i16() { return static_cast<int16_t>(u16()); }
};
}  // namespace

size_t encode_sens(const Sens& s, uint8_t* out) {
  uint8_t b[kSensLen];
  W w{b, 0};
  w.i16(s.flow_x_mrad); w.i16(s.flow_y_mrad); w.u8(s.quality); w.u8(s.flags); w.u16(s.range_mm); w.u16(s.seq);
  return encode(SENS, b, kSensLen, out);
}
size_t encode_cmd(const Cmd& c, uint8_t* out) {
  uint8_t b[kCmdLen];
  W w{b, 0};
  w.u8(c.op); w.i16(c.a); w.i16(c.b); w.i16(c.c); w.i16(c.yaw); w.u16(c.seq);
  return encode(CMD, b, kCmdLen, out);
}
size_t encode_telem(const Telem& t, uint8_t* out) {
  uint8_t b[kTelemLen];
  W w{b, 0};
  w.u8(t.mode); w.u8(t.state); w.u16(t.flags);
  for (int16_t v : t.pos_cm) w.i16(v);
  for (int16_t v : t.vel_cms) w.i16(v);
  w.i16(t.yaw_mrad); w.u16(t.vbat_mv); w.u16(t.cmd_seq);
  return encode(TELEM, b, kTelemLen, out);
}
bool decode_sens(const uint8_t* p, size_t len, Sens& s) {
  if (len != kSensLen) return false;
  R r{p, 0};
  s.flow_x_mrad = r.i16(); s.flow_y_mrad = r.i16(); s.quality = r.u8(); s.flags = r.u8(); s.range_mm = r.u16(); s.seq = r.u16();
  return true;
}
bool decode_cmd(const uint8_t* p, size_t len, Cmd& c) {
  if (len != kCmdLen) return false;
  R r{p, 0};
  c.op = r.u8(); c.a = r.i16(); c.b = r.i16(); c.c = r.i16(); c.yaw = r.i16(); c.seq = r.u16();
  return c.op <= OP_HOLD;
}
bool decode_telem(const uint8_t* p, size_t len, Telem& t) {
  if (len != kTelemLen) return false;
  R r{p, 0};
  t.mode = r.u8(); t.state = r.u8(); t.flags = r.u16();
  for (int16_t& v : t.pos_cm) v = r.i16();
  for (int16_t& v : t.vel_cms) v = r.i16();
  t.yaw_mrad = r.i16(); t.vbat_mv = r.u16(); t.cmd_seq = r.u16();
  return true;
}

bool Parser::push(uint8_t b) {
  switch (st_) {
    case S0:
      if (b == kSync0) st_ = S1;
      break;
    case S1:
      st_ = b == kSync1 ? TYPE : (b == kSync0 ? S1 : S0);
      break;
    case TYPE:
      type_ = b;
      st_ = LEN;
      break;
    case LEN:
      if (b > kMaxPayload) { ++len_err_; st_ = b == kSync0 ? S1 : S0; break; }
      len_ = b;
      i_ = 0;
      st_ = len_ ? PAY : C0;
      break;
    case PAY:
      buf_[i_++] = b;
      if (i_ >= len_) st_ = C0;
      break;
    case C0:
      crc_lo_ = b;
      st_ = C1;
      break;
    case C1: {
      st_ = S0;
      uint8_t hdr[2] = {type_, static_cast<uint8_t>(len_)};
      uint16_t crc = crc16(hdr, 2);
      crc = crc16(buf_, len_, crc);
      if ((crc & 0xFF) == crc_lo_ && (crc >> 8) == b) { ++good_; return true; }
      ++crc_err_;
      break;
    }
  }
  return false;
}

}  // namespace link

}  // namespace lite

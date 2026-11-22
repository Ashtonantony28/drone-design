// NF-3D Lite v3 companion link: framed serial protocol between the XIAO ESP32-S3 Sense
// companion board and the flight board (UART, 115200), and between the XIAO and the laptop
// (the same frames inside UDP datagrams). No dependencies, so the companion sketch and the
// laptop's Python twin (lite/ground/link.py) share one definition.
#pragma once

#include <cstddef>
#include <cstdint>

namespace lite {

// Frame: 0xA5 0x5A type len payload[len] crc_lo crc_hi
// crc = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len, payload. Little endian.
namespace link {
constexpr uint8_t kSync0 = 0xA5, kSync1 = 0x5A;
constexpr size_t kMaxPayload = 32;
constexpr size_t kOverhead = 6;  // sync x2, type, len, crc x2
enum Type : uint8_t {
  SENS = 0x01,   // XIAO -> FC: flow + range, 100 Hz
  CMD = 0x02,    // XIAO -> FC: laptop command (relayed), ~15 Hz
  TELEM = 0x81,  // FC -> XIAO: state for the laptop, 25 Hz
};
enum SensFlags : uint8_t { FLOW_OK = 1u << 0, RANGE_OK = 1u << 1 };
enum CmdOp : uint8_t {
  OP_VEL = 0,   // a, b, c = forward, left, up velocity (cm/s, heading frame); yaw = rate (mrad/s, + = CCW)
  OP_GOTO = 1,  // a, b, c = forward, left, up displacement (cm, heading frame), then hold
  OP_LAND = 2,
  OP_HOLD = 3,  // stop and hold here
};

struct Sens {   // 10 bytes
  int16_t flow_x_mrad = 0, flow_y_mrad = 0;  // rad/s * 1000, nf::FlowSample convention
  uint8_t quality = 0;                       // 0..255
  uint8_t flags = 0;                         // SensFlags
  uint16_t range_mm = 0;
  uint16_t seq = 0;
};
struct Cmd {    // 11 bytes
  uint8_t op = OP_HOLD;
  int16_t a = 0, b = 0, c = 0;
  int16_t yaw = 0;
  uint16_t seq = 0;
};
struct Telem {  // 22 bytes
  uint8_t mode = 0, state = 0;
  uint16_t flags = 0;
  int16_t pos_cm[3] = {0, 0, 0};
  int16_t vel_cms[3] = {0, 0, 0};
  int16_t yaw_mrad = 0;
  uint16_t vbat_mv = 0;
  uint16_t cmd_seq = 0;  // last CMD applied
};
constexpr size_t kSensLen = 10, kCmdLen = 11, kTelemLen = 22;

uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF);
// Encoders return the frame length written into out (>= kOverhead + payload).
size_t encode(uint8_t type, const uint8_t* payload, size_t len, uint8_t* out);
size_t encode_sens(const Sens& s, uint8_t* out);
size_t encode_cmd(const Cmd& c, uint8_t* out);
size_t encode_telem(const Telem& t, uint8_t* out);
bool decode_sens(const uint8_t* p, size_t len, Sens& s);
bool decode_cmd(const uint8_t* p, size_t len, Cmd& c);
bool decode_telem(const uint8_t* p, size_t len, Telem& t);

// Byte-at-a-time frame parser (resynchronises on garbage, no heap).
class Parser {
 public:
  // Returns true when a complete frame with a good CRC has just been received.
  bool push(uint8_t b);
  uint8_t type() const { return type_; }
  const uint8_t* payload() const { return buf_; }
  size_t len() const { return len_; }
  uint32_t good() const { return good_; }
  uint32_t crc_errors() const { return crc_err_; }
  uint32_t len_errors() const { return len_err_; }

 private:
  enum St : uint8_t { S0, S1, TYPE, LEN, PAY, C0, C1 } st_ = S0;
  uint8_t type_ = 0, buf_[kMaxPayload] = {};
  size_t len_ = 0, i_ = 0;
  uint16_t crc_ = 0;
  uint8_t crc_lo_ = 0;
  uint32_t good_ = 0, crc_err_ = 0, len_err_ = 0;
};
}  // namespace link

}  // namespace lite

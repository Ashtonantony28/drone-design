"""Python twin of lite/firmware/libraries/lite_fc/src/lite/link.hpp (the companion protocol).

Frame: A5 5A type len payload[len] crc_lo crc_hi; CRC-16/CCITT-FALSE over type, len, payload.
The laptop sends CMD frames to the XIAO over UDP (port 14560); the XIAO relays them to the
flight board and sends back TELEM (flight board state) and, at 10 Hz, SENS (sensor health).
lite/ground/tests check every encoder byte-for-byte against the C++ (tests/link_vectors.txt).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

SYNC = b"\xA5\x5A"
MAX_PAYLOAD = 32
SENS, CMD, TELEM = 0x01, 0x02, 0x81
FLOW_OK, RANGE_OK = 1, 2
OP_VEL, OP_GOTO, OP_LAND, OP_HOLD = 0, 1, 2, 3
MODES = ["ANGLE", "POS_HOLD", "ALT_HOLD", "LAND", "AUTO"]
STATES = ["CALIBRATING", "DISARMED", "ARMED"]


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(typ: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes((typ, len(payload))) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


def _i16(v: float) -> int:
    return max(-32768, min(32767, int(round(v))))


@dataclass
class Sens:
    flow_x_mrad: int = 0
    flow_y_mrad: int = 0
    quality: int = 0
    flags: int = 0
    range_mm: int = 0
    seq: int = 0
    FMT = "<hhBBHH"

    def pack(self) -> bytes:
        return encode(SENS, struct.pack(self.FMT, self.flow_x_mrad, self.flow_y_mrad, self.quality, self.flags,
                                        self.range_mm, self.seq & 0xFFFF))


@dataclass
class Cmd:
    op: int = OP_HOLD
    a: int = 0
    b: int = 0
    c: int = 0
    yaw: int = 0
    seq: int = 0
    FMT = "<BhhhhH"

    def pack(self) -> bytes:
        return encode(CMD, struct.pack(self.FMT, self.op, _i16(self.a), _i16(self.b), _i16(self.c), _i16(self.yaw),
                                       self.seq & 0xFFFF))

    @staticmethod
    def velocity(fwd: float, left: float, up: float, yaw_rate: float, seq: int = 0) -> "Cmd":
        """m/s heading frame, rad/s (+ = counter-clockwise from above)."""
        return Cmd(OP_VEL, _i16(fwd * 100), _i16(left * 100), _i16(up * 100), _i16(yaw_rate * 1000), seq)

    def velocity_si(self):
        return self.a * 0.01, self.b * 0.01, self.c * 0.01, self.yaw * 0.001


@dataclass
class Telem:
    mode: int = 0
    state: int = 0
    flags: int = 0
    pos_cm: list = field(default_factory=lambda: [0, 0, 0])
    vel_cms: list = field(default_factory=lambda: [0, 0, 0])
    yaw_mrad: int = 0
    vbat_mv: int = 0
    cmd_seq: int = 0
    FMT = "<BBHhhhhhhhHH"

    def pack(self) -> bytes:
        return encode(TELEM, struct.pack(self.FMT, self.mode, self.state, self.flags, *self.pos_cm, *self.vel_cms,
                                         self.yaw_mrad, self.vbat_mv, self.cmd_seq))

    @property
    def mode_name(self) -> str:
        return MODES[self.mode] if self.mode < len(MODES) else "?"


_DECODE = {SENS: (Sens, 10), CMD: (Cmd, 11), TELEM: (Telem, 22)}


def decode(typ: int, payload: bytes):
    """Payload -> Sens / Cmd / Telem, or None (unknown type, wrong length, bad op)."""
    if typ not in _DECODE:
        return None
    cls, n = _DECODE[typ]
    if len(payload) != n:
        return None
    v = struct.unpack(cls.FMT, payload)
    if cls is Telem:
        return Telem(v[0], v[1], v[2], list(v[3:6]), list(v[6:9]), v[9], v[10], v[11])
    obj = cls(*v)
    if cls is Cmd and obj.op > OP_HOLD:
        return None
    return obj


class Parser:
    """Byte-stream parser, same state machine as link::Parser. feed(bytes) -> list of decoded frames."""

    def __init__(self):
        self.buf = bytearray()
        self.good = self.crc_errors = 0

    def feed(self, data: bytes) -> list:
        out = []
        self.buf += data
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1 if self.buf[-1:] == b"\xA5" else len(self.buf)]
                return out
            del self.buf[:i]
            if len(self.buf) < 4:
                return out
            n = self.buf[3]
            if n > MAX_PAYLOAD:
                del self.buf[:2]
                continue
            if len(self.buf) < n + 6:
                return out
            body = bytes(self.buf[2:4 + n])
            crc = struct.unpack("<H", self.buf[4 + n:6 + n])[0]
            if crc16(body) == crc:
                obj = decode(body[0], body[2:])
                if obj is not None:
                    out.append(obj)
                self.good += 1
                del self.buf[:6 + n]
            else:
                self.crc_errors += 1
                del self.buf[:2]

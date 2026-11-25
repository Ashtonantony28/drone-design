"""lite/ground/link.py: frames byte-identical to the C++ (same vectors as
lite/firmware/tests/test_autonomy.cpp link_vectors_match_python), parser resync / fuzz."""
import random

from lite.ground import link as L

VEC = {
    "sens": (L.Sens(-1234, 32000, 201, 3, 3999, 65535), "a55a010a2efb007dc9039f0fffffbec0"),
    "cmd": (L.Cmd(1, 150, -75, 20, -300, 42), "a55a020b019600b5ff1400d4fe2a00925e"),
    "telem": (L.Telem(4, 2, 0x0401, [-12, 340, 150], [0, 0, -40], 3141, 3912, 42),
              "a55a811604020104f4ff5401960000000000d8ff450c480f2a007d4b"),
}


def test_crc_check_value():
    assert L.crc16(b"123456789") == 0x29B1


def test_vectors_match_cpp():
    for name, (obj, hx) in VEC.items():
        assert obj.pack().hex() == hx, name


def test_roundtrip():
    for obj, _ in VEC.values():
        got = L.Parser().feed(obj.pack())
        assert got == [obj]


def test_velocity_helper_units():
    c = L.Cmd.velocity(0.4, -0.25, 0.1, -0.5, seq=9)
    assert (c.op, c.a, c.b, c.c, c.yaw, c.seq) == (L.OP_VEL, 40, -25, 10, -500, 9)
    assert c.velocity_si() == (0.4, -0.25, 0.1, -0.5)
    big = L.Cmd.velocity(1e6, 0, 0, 0)
    assert big.a == 32767                          # saturates, never wraps


def test_parser_fuzz():
    rnd = random.Random(4)
    p = L.Parser()
    sent, got = [], []
    for i in range(3000):
        junk = bytes(rnd.randrange(256) for _ in range(rnd.randrange(10)))
        got += p.feed(junk)
        s = L.Sens(rnd.randrange(-3000, 3000), 0, 100, 3, rnd.randrange(4000), i)
        fr = bytearray(s.pack())
        if rnd.random() < 0.1:
            fr[2 + rnd.randrange(len(fr) - 2)] ^= 1 << rnd.randrange(8)
        else:
            sent.append(s)
        # split the frame at a random point: datagrams / UART reads can cut anywhere
        k = rnd.randrange(len(fr))
        got += p.feed(bytes(fr[:k]))
        got += p.feed(bytes(fr[k:]))
    good = [g for g in got if isinstance(g, L.Sens)]
    assert all(g in sent for g in good)             # nothing invented
    assert len(good) >= len(sent) - 30              # a corrupted length byte may swallow a neighbour


def test_bad_op_rejected():
    raw = bytearray(L.Cmd(L.OP_HOLD).pack())
    raw[4] = 9                                      # op 9
    body = bytes(raw[2:-2])
    raw[-2:] = L.crc16(body).to_bytes(2, "little")
    assert L.Parser().feed(bytes(raw)) == []

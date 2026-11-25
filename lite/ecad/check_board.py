#!/usr/bin/env python3
"""Checks for the Lite flight controller (lite/ecad): pin map, motor switches, power path,
mill rules, board size and holes.

    layout/run.sh dump drc            # refresh layout/build/{board,drc}.json from the .kicad_pcb
    python3 -m pytest check_board.py -q

Everything is read from the geometry dump written by layout/dump.py (mm from the board
centre, x right, y down, seen from the top) and from KiCad's own DRC report, so pytest
does not need KiCad.  The limits are the mill rules of MILLING.md; never loosen one to
make the layout pass.
"""
import json
import math
import re
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "layout"))
sys.path.insert(0, str(HERE / "lib"))
import geometry as G  # noqa: E402  (layout/geometry.py: board outline, holes, rule constants)
from gen_lib import SYMBOLS  # noqa: E402  (lib/gen_lib.py: ESP32 MiniKit pad -> pin name)

DUMP = HERE / "layout/build/board.json"
DRC = HERE / "layout/build/drc.json"
MILLING = HERE / "MILLING.md"
TOL = 0.011

# --- ESP32 rules (ESP32 datasheet / technical reference) --------------------------------
STRAPPING = {0, 2, 5, 12, 15}
INPUT_ONLY = set(range(34, 40))
FLASH = set(range(6, 12))
ADC1 = set(range(32, 40))
MOTORS = ("M1", "M2", "M3", "M4")
# Betaflight QUAD-X corners in board coordinates (nose = +x, drone right = +y)
MOTOR_CORNER = {"M1": (-1, 1), "M2": (1, 1), "M3": (-1, -1), "M4": (1, -1)}


@pytest.fixture(scope="module")
def board():
    if not DUMP.exists():
        pytest.skip("no %s: run layout/run.sh dump" % DUMP)
    return json.loads(DUMP.read_text())


@pytest.fixture(scope="module")
def fps(board):
    return {f["ref"]: f for f in board["footprints"]}


def pads(fp):
    return {p["number"]: p for p in fp["pads"]}


def net(fps, ref, pad):
    return pads(fps[ref])[pad]["net"]


def gpio_of(fps):
    """net -> GPIO number for every ESP32 pad that carries a net."""
    names = dict(SYMBOLS["ESP32_MiniKit"][1])
    out = {}
    for num, p in pads(fps["U1"]).items():
        m = re.fullmatch(r"IO(\d+)", names[num])
        if m and p["net"] and ".NC" not in p["net"]:
            out[p["net"]] = int(m.group(1))
    return out


# --- pin map -----------------------------------------------------------------------------

def test_esp32_power_pins(fps):
    names = dict(SYMBOLS["ESP32_MiniKit"][1])
    by_name = {names[n]: p["net"] for n, p in pads(fps["U1"]).items()}
    assert by_name["3V3"] == "3V3"
    assert by_name["GND"] == "GND"
    assert ".NC" in by_name["5V"], "the module's 5V pin must stay unconnected (USB power only)"


def test_motor_pins(fps):
    g = gpio_of(fps)
    for m in MOTORS:
        io = g.get(m + "_PWM")
        assert io is not None, "%s_PWM is not on the ESP32" % m
        assert io not in STRAPPING, "%s on strapping pin GPIO%d" % (m, io)
        assert io not in INPUT_ONLY, "%s on input-only GPIO%d" % (m, io)
        assert io not in FLASH and io < 34, "%s on GPIO%d, which cannot drive LEDC PWM" % (m, io)
    assert len({g[m + "_PWM"] for m in MOTORS}) == 4


def test_i2c_and_imu_pins(fps):
    g = gpio_of(fps)
    assert g["I2C_SDA"] == 21
    assert g["I2C_SCL"] == 22
    assert net(fps, "U2", "3") == "I2C_SCL" and net(fps, "U2", "4") == "I2C_SDA"
    io = g["IMU_INT"]
    assert io not in STRAPPING and io not in FLASH
    assert net(fps, "U2", "8") == "IMU_INT"


def test_battery_sense_on_adc1(fps):
    g = gpio_of(fps)
    assert g["VBAT_SENSE"] in ADC1, "Wi-Fi blocks ADC2; the battery divider must use ADC1"


def test_led_pin(fps):
    g = gpio_of(fps)
    io = g["LED"]
    assert io not in STRAPPING and io not in INPUT_ONLY and io not in FLASH
    assert {net(fps, "R11", "1"), net(fps, "R11", "2")} == {"LED", "LED_A"}
    assert "330" in fps["R11"]["value"]
    assert net(fps, "D5", "2") == "LED_A" and net(fps, "D5", "1") == "GND"


def test_no_pin_used_twice(fps):
    g = gpio_of(fps)
    assert len(set(g.values())) == len(g)


# --- motor switches ----------------------------------------------------------------------

@pytest.mark.parametrize("m", MOTORS)
def test_motor_switch(fps, m):
    n = int(m[1])
    q, rg, rpd, d = "Q%d" % n, "R%d" % n, "R%d" % (n + 4), "D%d" % n
    gate, drain, pwm = net(fps, q, "1"), net(fps, q, "3"), m + "_PWM"
    assert "AO3400" in fps[q]["value"]
    assert net(fps, q, "2") == "GND", "low-side switch: source to GND"
    assert {net(fps, rg, "1"), net(fps, rg, "2")} == {pwm, gate}, "gate resistor between PWM and gate"
    assert re.match(r"100R?\b", fps[rg]["value"]), fps[rg]["value"]
    assert {net(fps, rpd, "1"), net(fps, rpd, "2")} == {gate, "GND"}, "pull-down from gate to GND"
    assert fps[rpd]["value"].startswith("10k")
    assert net(fps, d, "1") == "VBAT" and net(fps, d, "2") == drain, "flyback: K to VBAT, A to the drain"
    assert "B5819W" in fps[d]["value"], "SOD-123 Schottky: low enough for the battery under the board"
    assert net(fps, m, "1") == "VBAT" and net(fps, m, "2") == drain, "motor + to VBAT, - to the drain"
    assert drain == m + "_N"


@pytest.mark.parametrize("m", MOTORS)
def test_motor_pads_at_corners(fps, m):
    sx, sy = MOTOR_CORNER[m]
    for p in fps[m]["pads"]:
        x, y = p["at"]
        assert x * sx > 15 and y * sy > 3, "%s pad at (%.1f, %.1f) is not in its corner" % (m, x, y)
        assert p["drill"] >= G.DRILL_MIN


# --- power path --------------------------------------------------------------------------

def test_power_path(fps):
    assert {net(fps, "J1", "1"), net(fps, "J1", "2")} == {"BAT_N", "BAT_P"}
    assert net(fps, "J1", "2") == "BAT_P"
    assert net(fps, "U3", "1") == "BAT_P" and net(fps, "U3", "2") == "BAT_N" and net(fps, "U3", "3") == "GND"
    assert net(fps, "SW1", "2") == "BAT_P", "switch common on the module output (B+ = OUT+)"
    assert "VBAT" in (net(fps, "SW1", "1"), net(fps, "SW1", "3")), "one throw of the switch feeds VBAT"
    assert {net(fps, "C1", "1"), net(fps, "C1", "2")} == {"VBAT", "GND"} and net(fps, "C1", "1") == "VBAT"
    assert "470u" in fps["C1"]["value"]
    assert {net(fps, "JP1", "1"), net(fps, "JP1", "2")} == {"GND", "BAT_N"}


def test_ldo(fps):
    assert [net(fps, "U4", n) for n in "1235"] == ["VBAT", "GND", "VBAT", "3V3"]
    assert {net(fps, "C2", "1"), net(fps, "C2", "2")} == {"VBAT", "GND"} and "10u" in fps["C2"]["value"]
    assert {net(fps, "C3", "1"), net(fps, "C3", "2")} == {"3V3", "GND"} and "10u" in fps["C3"]["value"]


def test_imu_decoupling(fps):
    vcc, gnd = pads(fps["U2"])["1"], pads(fps["U2"])["2"]
    c = pads(fps["C4"])
    assert {c["1"]["net"], c["2"]["net"]} == {vcc["net"], "GND"} and gnd["net"] == "GND"
    assert "100n" in fps["C4"]["value"]
    for cp in c.values():
        ref = vcc if cp["net"] == vcc["net"] else gnd
        assert math.dist(cp["at"], ref["at"]) < 3.5, "C4 must sit at the IMU supply pins"


def test_battery_divider(fps):
    assert {net(fps, "R9", "1"), net(fps, "R9", "2")} == {"VBAT", "VBAT_SENSE"} and fps["R9"]["value"].startswith("100k")
    assert {net(fps, "R10", "1"), net(fps, "R10", "2")} == {"VBAT_SENSE", "GND"} and fps["R10"]["value"].startswith("47k")
    assert 4.2 * 47 / 147 < 1.5  # inside the ESP32 ADC's 11 dB range with margin


# --- outline and holes -------------------------------------------------------------------

def test_board_size(board):
    # from the outline's own coordinates (the dump's bounding box includes the line width)
    xs = [p[0] for e in board["edges"] for p in (e["start"], e["end"])]
    ys = [p[1] for e in board["edges"] for p in (e["start"], e["end"])]
    w, h = max(xs) - min(xs), max(ys) - min(ys)
    assert w <= 50 + TOL and h <= 50 + TOL
    assert abs(w - G.BOARD_W) < 0.05 and abs(h - G.BOARD_H) < 0.05
    assert sum(1 for e in board["edges"] if e["center"]) == 4, "rounded corners"


def test_mounting_holes(fps):
    holes = sorted(tuple(round(c, 2) for c in fps["H%d" % i]["pads"][0]["at"]) for i in range(1, 5))
    assert holes == sorted((sx * 19.0, sy * 15.0) for sx in (-1, 1) for sy in (-1, 1))
    for i in range(1, 5):
        p = fps["H%d" % i]["pads"][0]
        assert p["npth"] and abs(p["drill"] - G.HOLE_D) < TOL


# --- mill rules --------------------------------------------------------------------------

def in_box(pt, box, grow=0.0):
    return box[0] - grow <= pt[0] <= box[2] + grow and box[1] - grow <= pt[1] <= box[3] + grow


def test_track_widths(board, fps):
    u4 = fps["U4"]["courtyard"]
    assert board["tracks"], "board not routed"
    for t in board["tracks"]:
        assert t["width"] >= G.TRACK_MIN - TOL, t
        if t["net"] in G.POWER_NETS:
            near_u4 = in_box(t["start"], u4) or in_box(t["end"], u4)
            if not near_u4:
                assert t["width"] >= G.POWER_TRACK_MIN - TOL, t


def test_drills(board):
    for f in board["footprints"]:
        for p in f["pads"]:
            if p["drill"]:
                assert p["drill"] >= G.DRILL_MIN - TOL, (f["ref"], p["number"], p["drill"])
    for v in board["vias"]:
        assert v["drill"] >= 1.0 - TOL and v["dia"] >= 2.0 - TOL, v


def test_smd_parts_are_millable(board):
    allowed = ("SOT-23_Mill", "SOT-23-5_Mill", "R_1206_Mill", "C_1206_Mill", "D_SMA_SOD-123_Mill",
               "SolderJumper_2_Open_Mill")
    for f in board["footprints"]:
        if all(p["drill"] == 0 for p in f["pads"]) and f["pads"]:
            assert f["footprint"] in allowed, (f["ref"], f["footprint"])
            assert f["side"] == "B", "%s: SMD parts sit on the bottom copper" % f["ref"]


def tht_or_via_at(board, pt):
    for f in board["footprints"]:
        for p in f["pads"]:
            if p["drill"] and not p["npth"] and in_box(pt, p["box"]):
                return True
    return any(math.dist(v["at"], pt) <= v["dia"] / 2 for v in board["vias"])


def top_jumpers(board):
    """Connected runs of F.Cu track: each one is a top-side jumper."""
    segs = [t for t in board["tracks"] if t["layer"] == "F.Cu"]
    groups = []
    for t in segs:
        ends = {tuple(t["start"]), tuple(t["end"])}
        hit = [g for g in groups if g["pts"] & ends]
        merged = {"pts": set(ends), "segs": [t]}
        for g in hit:
            merged["pts"] |= g["pts"]
            merged["segs"] += g["segs"]
            groups.remove(g)
        groups.append(merged)
    return groups


def test_top_jumpers_end_on_holes(board):
    for g in top_jumpers(board):
        deg = {}
        for t in g["segs"]:
            for e in (tuple(t["start"]), tuple(t["end"])):
                deg[e] = deg.get(e, 0) + 1
        for e, k in deg.items():
            if k == 1:
                assert tht_or_via_at(board, e), "top track end %s is not on a hole (THT pad or wire via)" % (e,)


def test_top_jumpers_listed(board):
    """MILLING.md must list every top-side track and wire via the layout has."""
    text = MILLING.read_text()
    m = re.search(r"top-side tracks:\s*(\d+)", text)
    v = re.search(r"wire vias:\s*(\d+)", text)
    assert m and v, "MILLING.md must state 'top-side tracks: N' and 'wire vias: N'"
    assert int(m.group(1)) == len(top_jumpers(board))
    assert int(v.group(1)) == len(board["vias"])


def test_drc_clean():
    if not DRC.exists():
        pytest.skip("no %s: run layout/run.sh drc" % DRC)
    d = json.loads(DRC.read_text())
    errors = [v for v in d.get("violations", []) if v.get("severity") == "error"]
    assert not errors, ["%s: %s" % (v["type"], v["description"]) for v in errors[:10]]
    assert not d.get("unconnected_items"), "%d unconnected" % len(d["unconnected_items"])


def test_drc_rules_in_project():
    pro = json.loads(next((HERE / "boards/lite_fc/layout/lite_fc").glob("*.kicad_pro")).read_text())
    r = pro["board"]["design_settings"]["rules"]
    assert r["min_clearance"] >= G.CLEARANCE - TOL
    assert r["min_track_width"] >= G.TRACK_MIN - TOL
    assert r["min_through_hole_diameter"] >= G.DRILL_MIN - TOL
    assert r["min_copper_edge_clearance"] >= G.EDGE_CLEARANCE - TOL
    classes = {c["name"]: c for c in pro["net_settings"]["classes"]}
    assert classes["Power"]["track_width"] >= G.POWER_TRACK_MIN - TOL

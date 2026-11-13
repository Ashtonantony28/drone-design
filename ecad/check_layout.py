#!/usr/bin/env python3
"""Layout checks for nf_fc against neuroflight-fc/hw/DESIGN.md sections 1, 4 and 5.

    layout/run.sh dump drc          # refresh layout/build/{board,drc}.json from the .kicad_pcb
    python -m pytest check_layout.py -q

The checks read the geometry dump written by layout/dump.py (mm, origin at the
board centre, x right, y down, seen from the top) and KiCad's own DRC report,
so pytest itself does not need KiCad.  Distances are copper-to-copper (pad
bounding boxes, track centre lines minus half the width), not centre-to-centre.

Limits come from DESIGN.md; where it gives none the limit is stated here as
ours.  Never loosen one to make a layout pass: fix the layout or report it.
"""
import json
import math
import os
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "layout"))
import impedance  # noqa: E402  (layout/impedance.py: stackup, pair impedance)
import netlen  # noqa: E402  (layout/netlen.py: pad-to-pad copper length)
import widen  # noqa: E402  (layout/widen.py: copper spacing statistics)

BOARD = HERE / "boards/nf_fc/layout/nf_fc/nf_fc.kicad_pcb"
DUMP = HERE / "layout/build/board.json"
DRC = HERE / "layout/build/drc.json"

# --- limits -----------------------------------------------------------------
BOARD_SIZE = 36.0          # DESIGN.md 1: 36 x 36 mm
HOLE_XY = 15.25            # DESIGN.md 1: holes at (+-15.25, +-15.25)
HOLE_DRILL = 4.0           # DESIGN.md 1: 4.0 mm for M3 grommets
HOLE_KEEPOUT_R = 3.5       # DESIGN.md 1: ~7 mm keep-out circle, all layers
IMU_TO_BUCK_MIN = 10.0     # DESIGN.md 4: IMUs >= 10 mm from inductor and switch node
IMU_TO_J2_DSHOT_MIN = 8.0  # DESIGN.md 4: IMUs >= 8 mm from J2 and the DShot traces
ISLAND_RING = 1.0          # DESIGN.md 4: 1 mm keep-out ring around the island
MCU_DECAP_MAX = 1.0        # DESIGN.md 5.1: 100 nF at every VDD pin within 1 mm
IC_DECAP_MAX = 2.0         # ours: other ICs, capacitor pad to supply pin
DSHOT_MAX_LEN = 25.0       # DESIGN.md 5.5
STITCH_PITCH_MAX = 4.0     # DESIGN.md 5.4 says "every ~3 mm"; we allow up to 4 mm
TOL = 0.011                # mm, coordinate rounding
# Pre-fabrication margins (ours; JLC's standard capability is 0.127 mm, 0.1 mm is its limit)
TRACK_MIN = 0.1            # DESIGN.md 1: nothing below the fab's minimum
TRACK_STD = 0.127          # width and space everywhere the layout has room for it
NARROW_SHARE_MAX = 0.18    # share of track length narrower than TRACK_STD (0.78 before layout/widen.py)
TIGHT_SHARE_MAX = 0.10     # share of track length with foreign copper nearer than TRACK_STD
QSPI_NETS = ("QSPI_CLK", "QSPI_NCS", "QSPI_IO0", "QSPI_IO1", "QSPI_IO2", "QSPI_IO3")
QSPI_SPREAD_MAX = 4.0      # DESIGN.md 5.7: the 6 lines matched to +-2 mm
FLASH = "FLASH.U5.X"
# L4 power pours: zone name -> (net, smallest filled area in mm2)
POWER_POURS = {"L4_VBAT_IN": ("VBAT", 20.0), "L4_VBAT_HF": ("VBAT", 4.0), "L4_5V_OUT": ("5V", 9.0)}

IMUS = ("IMU.U2.X", "IMU.U3.X")
BUCK_SW_NET = "PWR.BUCK_SW"
INDUCTOR = "PWR.L1.X"
BUCK = "PWR.U6.X"
ESC_CONN = "CONN.J2.X"
MCU = "MCU.U1.X"
# MCU pin -> MCU net of the four DShot outputs, and the net after the 33R resistor
DSHOT = {"MOTOR0": ("CONN.R_M1.R", "ESC_M1"), "MOTOR1": ("CONN.R_M2.R", "CONN.ESC_M2"),
         "MOTOR2": ("CONN.R_M3.R", "CONN.ESC_M3"), "MOTOR3": ("CONN.R_M4.R", "CONN.ESC_M4")}
# nets allowed on and around the IMU island
ISLAND_NET_PREFIXES = ("3V3_IMU", "GND", "IMUA_", "IMUB_", "IMU.")
# high-di/dt nets that must not pass under the island on any layer (DESIGN.md 4)
NOISY_NETS = ("VBAT", "5V", "PWR.BUCK_SW", "PWR.BUCK_BOOT", "MOTOR0", "MOTOR1", "MOTOR2", "MOTOR3",
              "ESC_M1", "CONN.ESC_M2", "CONN.ESC_M3", "CONN.ESC_M4", "USB_P", "USB_N", "QSPI_CLK")
# MCU supply pin -> capacitor that decouples it (DESIGN.md 5.1)
MCU_DECAPS = {
    "6": "MCU.C_VBAT_6.C", "11": "MCU.C_VDD_11.C", "21": "MCU.C_VDDA_100N.C", "27": "MCU.C_VDD_27.C",
    "50": "MCU.C_VDD_50.C", "75": "MCU.C_VDD_75.C", "100": "MCU.C_VDD_100.C",
    "48": "MCU.C_VCAP_48.C", "73": "MCU.C_VCAP_73.C",
}
# other ICs: (IC path, pin net) -> capacitor path
IC_DECAPS = {
    ("IMU.U2.X", "3V3_IMU"): ("IMU.C_U2_VDD_100N.C", "IMU.C_U2_VDDIO.C"),
    ("IMU.U3.X", "3V3_IMU"): ("IMU.C_U3_VDD.C", "IMU.C_U3_VDDIO.C"),
    ("BARO.U4.X", "3V3_D"): ("BARO.C_U4_VDD.C", "BARO.C_U4_VDDIO.C"),
    ("FLASH.U5.X", "3V3_D"): ("FLASH.C_U5_VCC.C",),
    ("PWR.U6.X", "VBAT"): ("PWR.C_VIN_HF1.C", "PWR.C_VIN_HF2.C"),
    ("PWR.U6.X", "PWR.BUCK_VCC"): ("PWR.C_VCC.C",),
    ("PWR.U8.X", "3V3_IMU"): ("PWR.C_U8_OUT1.C",),
    ("PWR.U8.X", "PWR.IMU_LDO_IN"): ("PWR.C_U8_IN.C",),
    ("PWR.U11.X", "3V3_D"): ("PWR.C_U11_IN.C",),
    ("PWR.U11.X", "1V2_FPGA"): ("PWR.C_U11_OUT.C",),
}


# --- geometry ---------------------------------------------------------------
def rect_rect(a, b):
    dx = max(a[0] - b[2], b[0] - a[2], 0.0)
    dy = max(a[1] - b[3], b[1] - a[3], 0.0)
    return math.hypot(dx, dy)


def point_rect(p, r):
    return rect_rect((p[0], p[1], p[0], p[1]), r)


def seg_points(a, b, step=0.05):
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]


def track_rect(t, r):
    """Copper distance from a track to a rectangle."""
    return max(0.0, min(point_rect(p, r) for p in seg_points(t["start"], t["end"])) - t["width"] / 2)


def via_rect(v, r):
    return max(0.0, point_rect(v["at"], r) - v["dia"] / 2)


def point_in_poly(p, poly):
    inside = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < (x2 - x1) * (p[1] - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def bbox(points):
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


# --- data -------------------------------------------------------------------
class Layout:
    def __init__(self, data):
        self.d = data
        self.fp = {f["path"]: f for f in data["footprints"]}
        self.tracks = data["tracks"]
        self.vias = data["vias"]
        self.zones = {z["name"]: z for z in data["zones"]}

    def pads(self, path, net=None):
        return [p for p in self.fp[path]["pads"] if net is None or p["net"] == net]

    def net_copper(self, nets):
        """(kind, object) for every pad, track and via on the given nets."""
        nets = set(nets)
        out = [("pad", p) for f in self.d["footprints"] for p in f["pads"] if p["net"] in nets]
        out += [("track", t) for t in self.tracks if t["net"] in nets]
        out += [("via", v) for v in self.vias if v["net"] in nets]
        return out

    @staticmethod
    def dist(item, rect):
        kind, o = item
        return rect_rect(o["box"], rect) if kind == "pad" else track_rect(o, rect) if kind == "track" else via_rect(o, rect)

    def island(self):
        """Bounding box of the IMU island = the L3 3V3_IMU pour under the IMUs,
        without the narrow neck that feeds it from the LDO."""
        imu = bbox([p["at"] for i in IMUS for p in self.pads(i)])
        pts = self.zones["L3_3V3_IMU"]["outline"]
        xs = sorted({p[0] for p in pts})
        ys = sorted({p[1] for p in pts})
        # largest axis-aligned rectangle of the outline that contains the IMUs
        best = None
        for x0 in xs:
            for x1 in xs:
                for y0 in ys:
                    for y1 in ys:
                        if x0 <= imu[0] and x1 >= imu[2] and y0 <= imu[1] and y1 >= imu[3]:
                            corners = [(x0 + 1e-3, y0 + 1e-3), (x1 - 1e-3, y0 + 1e-3), (x1 - 1e-3, y1 - 1e-3), (x0 + 1e-3, y1 - 1e-3)]
                            if all(point_in_poly(c, pts) for c in corners):
                                area = (x1 - x0) * (y1 - y0)
                                if best is None or area > best[0]:
                                    best = (area, (x0, y0, x1, y1))
        assert best, "no 3V3_IMU island under the IMUs"
        return best[1]


@pytest.fixture(scope="module")
def lay():
    assert DUMP.exists(), "run layout/run.sh dump drc first"
    assert DUMP.stat().st_mtime >= BOARD.stat().st_mtime, "layout/build/board.json is older than the board: run layout/run.sh dump drc"
    return Layout(json.loads(DUMP.read_text()))


def is_island_net(net):
    return any(net == p or (p.endswith(("_", ".")) and net.startswith(p)) for p in ISLAND_NET_PREFIXES)


# --- tests ------------------------------------------------------------------
def test_outline_and_mounting_holes(lay):
    """36 x 36 mm outline; four 4.0 mm NPTH at (+-15.25, +-15.25); no copper in the 7 mm circles."""
    e = lay.d["edge"]
    # the bounding box includes the 0.1 mm edge line width
    assert abs((e[2] - e[0]) - BOARD_SIZE) <= 0.11 and abs((e[3] - e[1]) - BOARD_SIZE) <= 0.11, e
    assert abs(e[0] + e[2]) <= TOL and abs(e[1] + e[3]) <= TOL, "outline not centred"
    holes = sorted(tuple(f["at"]) for f in lay.d["footprints"] if f["footprint"].startswith("MountingHole"))
    want = sorted((sx * HOLE_XY, sy * HOLE_XY) for sx in (-1, 1) for sy in (-1, 1))
    assert len(holes) == 4
    for got, exp in zip(holes, want):
        assert abs(got[0] - exp[0]) <= TOL and abs(got[1] - exp[1]) <= TOL, (got, exp)
    for f in lay.d["footprints"]:
        if f["footprint"].startswith("MountingHole"):
            assert len(f["pads"]) == 1 and f["pads"][0]["npth"] and abs(f["pads"][0]["drill"] - HOLE_DRILL) < 1e-6, f["ref"]
    bad = []
    for hx, hy in want:
        for f in lay.d["footprints"]:
            if f["footprint"].startswith("MountingHole"):
                continue
            for p in f["pads"]:
                if point_rect((hx, hy), p["box"]) < HOLE_KEEPOUT_R - TOL:
                    bad.append((f["ref"], p["number"]))
        for t in lay.tracks:
            if track_rect(t, (hx, hy, hx, hy)) < HOLE_KEEPOUT_R - TOL:
                bad.append(("track", t["net"], t["start"]))
        for v in lay.vias:
            if via_rect(v, (hx, hy, hx, hy)) < HOLE_KEEPOUT_R - TOL:
                bad.append(("via", v["net"], v["at"]))
    assert not bad, bad


def test_imu_to_buck_inductor_and_switch_node(lay):
    """Both IMUs >= 10 mm from L1 and from every piece of switch-node copper."""
    targets = [("pad", p) for p in lay.pads(INDUCTOR)] + lay.net_copper([BUCK_SW_NET])
    l1 = lay.fp[INDUCTOR]["courtyard"]
    sw_zones = [z for z in lay.d["zones"] if z["net"] == BUCK_SW_NET]
    for imu in IMUS:
        for pad in lay.pads(imu):
            d = min([lay.dist(t, pad["box"]) for t in targets] + [rect_rect(l1, pad["box"])]
                    + [rect_rect(bbox(z["outline"]), pad["box"]) for z in sw_zones])
            assert d >= IMU_TO_BUCK_MIN, (imu, pad["number"], round(d, 2))


def imu_clearances(lay):
    """Smallest copper distance from each IMU to J2 and to DShot copper."""
    dshot_nets = list(DSHOT) + [v[1] for v in DSHOT.values()]
    dshot = lay.net_copper(dshot_nets)
    assert sum(1 for k, _ in dshot if k == "track") >= 8, "DShot nets are not routed"
    j2 = [("pad", p) for p in lay.pads(ESC_CONN)]
    out = {}
    for imu in IMUS:
        boxes = [p["box"] for p in lay.pads(imu)]
        out[imu] = (min(lay.dist(t, b) for t in j2 for b in boxes), min(lay.dist(t, b) for t in dshot for b in boxes))
    return out, dshot


def test_imu_to_j2_and_dshot(lay):
    """DESIGN.md 4: both IMUs >= 8 mm from J2 and from the DShot traces
    (MCU pin, 33R resistor, trace, via and connector pin all count)."""
    got, dshot = imu_clearances(lay)
    for imu, (d_j2, d_dshot) in got.items():
        assert d_j2 >= IMU_TO_J2_DSHOT_MIN, (imu, "J2", round(d_j2, 2))
        assert d_dshot >= IMU_TO_J2_DSHOT_MIN, (imu, "DShot", round(d_dshot, 2))
    # DShot never runs under or beside the island on any layer
    isl = lay.island()
    for item in dshot:
        assert lay.dist(item, isl) >= ISLAND_RING, ("DShot copper at the island", item[1])


def test_imu_island(lay):
    """3V3_IMU island on L3 under both IMUs, a 1 mm ring around it free of
    foreign parts and tracks, and no noisy net under it on any layer."""
    isl = lay.island()
    z = lay.zones["L3_3V3_IMU"]
    assert z["layers"] == ["In2.Cu"] and z["filled"]["In2.Cu"]["outlines"] == 1 and z["filled"]["In2.Cu"]["area"] > 40
    for imu in IMUS:
        for p in lay.pads(imu):
            assert rect_rect(p["box"], isl) == 0 and isl[0] <= p["at"][0] <= isl[2] and isl[1] <= p["at"][1] <= isl[3]
    ring = (isl[0] - ISLAND_RING, isl[1] - ISLAND_RING, isl[2] + ISLAND_RING, isl[3] + ISLAND_RING)
    bad = []
    for f in lay.d["footprints"]:
        nets = {p["net"] for p in f["pads"] if p["net"]}
        if not nets or f["footprint"].startswith("MountingHole"):
            continue
        if any(n == "3V3_IMU" or n.startswith(("IMUA_", "IMUB_", "IMU.")) for n in nets):
            continue  # IMU circuit, its regulator output and its test points
        if f["courtyard"] and rect_rect(f["courtyard"], isl) < ISLAND_RING - TOL:
            bad.append((f["ref"], round(rect_rect(f["courtyard"], isl), 2)))
    for t in lay.tracks:
        if not is_island_net(t["net"]) and track_rect(t, isl) < ISLAND_RING - TOL:
            bad.append(("track", t["net"], t["layer"], t["start"]))
    for v in lay.vias:
        if not is_island_net(v["net"]) and via_rect(v, isl) < ISLAND_RING - TOL:
            bad.append(("via", v["net"], v["at"]))
    for kind, o in lay.net_copper(NOISY_NETS):
        if lay.dist((kind, o), ring) == 0:
            bad.append(("noisy", kind, o["net"]))
    # the neighbouring L3 pours stay out of the ring too
    for z in lay.d["zones"]:
        if z["keepout"] or z["name"] in ("L2_GND", "L3_3V3_IMU") or z["net"] in ("GND", "3V3_IMU"):
            continue
        pts = z["outline"]
        for i in range(len(pts)):
            near = [p for p in seg_points(pts[i], pts[(i + 1) % len(pts)], 0.25) if point_rect(p, isl) < ISLAND_RING - TOL]
            if near:
                bad.append((z["name"], [round(c, 2) for c in near[0]]))
    assert not bad, bad


def pad_gap(a, b):
    return rect_rect(a["box"], b["box"])


def test_mcu_decoupling_within_1mm(lay):
    """Every MCU VDD/VBAT/VDDA/VCAP pin has its capacitor's pad within 1 mm (copper to copper)."""
    for pin, cap in MCU_DECAPS.items():
        mcu_pad = [p for p in lay.pads(MCU) if p["number"] == pin][0]
        cap_pads = lay.pads(cap, mcu_pad["net"])
        assert cap_pads, (pin, cap, "capacitor is not on net", mcu_pad["net"])
        d = min(pad_gap(mcu_pad, c) for c in cap_pads)
        assert d <= MCU_DECAP_MAX, (pin, cap, round(d, 2))
    # no supply pin was left out of the table
    supply = {p["number"] for p in lay.pads(MCU) if p["net"] in ("3V3_D", "VDDA") or p["net"].startswith("MCU.VCAP")}
    assert supply - set(MCU_DECAPS) <= {"20"}, supply - set(MCU_DECAPS)  # 20 = VREF+, tied to VDDA at pin 21
    # VDDA filter parts and the bulk capacitor sit by the MCU
    u1 = lay.fp[MCU]["courtyard"]
    for path, lim in (("MCU.C_VDDA_1U.C", 1.0), ("MCU.FB2.FB", 3.0), ("MCU.C_VDD_BULK.C", 3.0)):
        assert rect_rect(lay.fp[path]["courtyard"], u1) <= lim, path


def test_ic_decoupling_within_2mm(lay):
    for (ic, net), caps in IC_DECAPS.items():
        pins = lay.pads(ic, net)
        assert pins, (ic, net)
        for cap in caps:
            cap_pads = lay.pads(cap, net)
            assert cap_pads, (cap, net)
            d = min(pad_gap(p, c) for p in pins for c in cap_pads)
            assert d <= IC_DECAP_MAX, (ic, net, cap, round(d, 2))


def dshot_lengths(lay):
    out = {}
    for mcu_net, (res, esc_net) in DSHOT.items():
        tr = [t for t in lay.tracks if t["net"] in (mcu_net, esc_net)]
        r = lay.pads(res)
        span = math.dist(r[0]["at"], r[1]["at"])
        out[mcu_net] = (sum(t["len"] for t in tr) + span, len(tr))
    return out


def test_dshot_routing(lay):
    """PE9/11/13/14 -> 33R -> J2: each <= 25 mm of copper, the resistor at the MCU pin."""
    for net, (length, n) in dshot_lengths(lay).items():
        assert n >= 2, (net, "not routed")
        assert length <= DSHOT_MAX_LEN, (net, round(length, 2))
    for mcu_net, (res, esc_net) in DSHOT.items():
        pin = lay.pads(MCU, mcu_net)[0]
        d = min(pad_gap(pin, p) for p in lay.pads(res, mcu_net))
        assert d <= 5.0, (res, "series resistor is not at the MCU pin", round(d, 2))  # ours
        assert lay.pads(ESC_CONN, esc_net), (esc_net, "does not reach J2")
        stub = sum(t["len"] for t in lay.tracks if t["net"] == mcu_net)
        assert stub <= 6.0, (mcu_net, "MCU-side stub", round(stub, 2))  # ours


def test_ground_plane_is_solid(lay):
    """L2 is one unbroken GND pour over the whole board with no tracks on it."""
    z = lay.zones["L2_GND"]
    f = z["filled"]["In1.Cu"]
    assert z["net"] == "GND" and z["layers"] == ["In1.Cu"]
    assert f["outlines"] == 1, "L2 GND is split into %d pieces" % f["outlines"]
    board_area = BOARD_SIZE ** 2 - (4 - math.pi) * 3.0 ** 2
    holes = 4 * math.pi * HOLE_KEEPOUT_R ** 2
    assert f["area"] >= 0.90 * (board_area - holes), f
    assert not [t for t in lay.tracks if t["layer"] == "In1.Cu"], "tracks on the GND plane layer"


def test_ground_stitching(lay):
    """GND vias along the board edges (pitch <= 4 mm between the hole keep-outs) and around the buck."""
    gnd = [v["at"] for v in lay.vias if v["net"] == "GND"]
    lim = BOARD_SIZE / 2
    span = HOLE_XY - HOLE_KEEPOUT_R  # edges between the mounting-hole keep-outs
    gaps = []
    for axis, sign in ((0, -1), (0, 1), (1, -1), (1, 1)):
        along = sorted(v[1 - axis] for v in gnd if abs(v[axis] - sign * lim) <= 1.5 and abs(v[1 - axis]) <= span)
        pts = [-span] + along + [span]
        worst = max(b - a for a, b in zip(pts, pts[1:]))
        if worst > STITCH_PITCH_MAX + TOL:
            gaps.append((("x", "y")[axis], sign, round(worst, 2)))
    assert not gaps, gaps
    cy = lay.fp[BUCK]["courtyard"]
    near = [v for v in gnd if rect_rect((v[0], v[1], v[0], v[1]), cy) <= 3.0]
    assert len(near) >= 8, len(near)


def test_placement_rules(lay):
    """MCU central on top; IMUs on top; buck and inductor on the bottom next to J2."""
    assert lay.fp[MCU]["side"] == "F" and math.hypot(*lay.fp[MCU]["at"]) <= 3.0
    for imu in IMUS:
        assert lay.fp[imu]["side"] == "F"
    j2 = lay.fp[ESC_CONN]
    for p in (BUCK, INDUCTOR):
        assert lay.fp[p]["side"] == "B" and j2["side"] == "B"
        assert rect_rect(lay.fp[p]["courtyard"], j2["courtyard"]) <= 12.0, p  # ours: same corner
    # IMU island and J2/buck in opposite corners
    ix, iy = lay.fp[IMUS[0]]["at"]
    for p in (BUCK, ESC_CONN):
        x, y = lay.fp[p]["at"]
        assert ix < 0 and iy < 0 and x > 0 and y > 0, p
    # nothing but the edge connectors overhangs the outline; no courtyards collide
    h = BOARD_SIZE / 2
    for f in lay.d["footprints"]:
        for p in f["pads"]:
            b = p["box"]
            assert b[0] >= -h + 0.3 and b[1] >= -h + 0.3 and b[2] <= h - 0.3 and b[3] <= h - 0.3, (f["ref"], p["number"])
    fps = [f for f in lay.d["footprints"] if f["courtyard"]]
    hits = []
    for i, a in enumerate(fps):
        for b in fps[i + 1:]:
            if a["side"] == b["side"] or "MountingHole" in a["footprint"] + b["footprint"]:
                ca, cb = a["courtyard"], b["courtyard"]
                ox, oy = min(ca[2], cb[2]) - max(ca[0], cb[0]), min(ca[3], cb[3]) - max(ca[1], cb[1])
                if ox > TOL and oy > TOL:
                    # mounting-hole courtyards are squares around a circle: test the circle instead
                    hole = a if "MountingHole" in a["footprint"] else b if "MountingHole" in b["footprint"] else None
                    if hole is not None:
                        other = b if hole is a else a
                        if point_rect(hole["at"], other["courtyard"]) >= HOLE_KEEPOUT_R:
                            continue
                    hits.append((a["ref"], b["ref"]))
    assert not hits, hits


def test_drc_clean():
    """kicad-cli pcb drc --schematic-parity is not available for a Zener design;
    the netlist side is covered by `pcb layout --check` (see layout/run.sh drc)."""
    assert DRC.exists(), "run layout/run.sh drc"
    assert DRC.stat().st_mtime >= BOARD.stat().st_mtime, "layout/build/drc.json is older than the board"
    r = json.loads(DRC.read_text())
    errors = [v for v in r.get("violations", []) if v["severity"] == "error"]
    assert not errors, "%d DRC errors, first: %s" % (len(errors), errors[:3])
    assert not r.get("unconnected_items"), "%d unconnected, first: %s" % (len(r["unconnected_items"]), r["unconnected_items"][:3])


def copper_z():
    """z of each copper layer (mm) from the board's stackup: a via counts with its height."""
    return impedance.copper_spans(impedance.read_stackup(BOARD))


def qspi_lengths(lay):
    """Copper length MCU pad -> flash pad of each QSPI line (shortest path: pull-up stubs do not count)."""
    z = copper_z()
    return {n: list(netlen.lengths(n, lay.d, (MCU, None), [(FLASH, None)], z).values())[0] for n in QSPI_NETS}


def test_qspi_length_matched(lay):
    """DESIGN.md 5.7: the six QSPI lines are length-matched to +-2 mm (MCU pad to flash pad, vias included)."""
    L = qspi_lengths(lay)
    assert all(v is not None for v in L.values()), L
    spread = max(L.values()) - min(L.values())
    assert spread <= QSPI_SPREAD_MAX, ({n: round(v, 2) for n, v in L.items()}, round(spread, 2))


def test_track_and_space_margin(lay):
    """No track under 0.1 mm; width and space of 0.127 mm or more except in the listed share
    of the track length (the MCU and FPGA escapes; `python check_layout.py` prints where)."""
    assert min(t["width"] for t in lay.tracks) >= TRACK_MIN - 1e-9
    s = widen.stats(lay.d)
    assert s["narrow"] <= NARROW_SHARE_MAX, "%.1f %% of the track length is under %.3f mm wide" % (100 * s["narrow"], TRACK_STD)
    assert s["tight_space"] <= TIGHT_SHARE_MAX, "%.1f %% of the track length has less than %.3f mm space" % (100 * s["tight_space"], TRACK_STD)


def test_power_pours(lay):
    """VBAT input and buck output are poured on L4; every pour is filled and in one piece per pad group."""
    for name, (net, area) in POWER_POURS.items():
        z = lay.zones.get(name)
        assert z is not None, name
        assert z["net"] == net and z["layers"] == ["B.Cu"], (name, z["net"], z["layers"])
        assert z["filled"]["B.Cu"]["area"] >= area, (name, z["filled"])
        # at least two pads of the net sit inside the pour outline: it joins parts, it is not a patch
        inside = [p for f in lay.d["footprints"] for p in f["pads"] if p["net"] == net and "B" in p["layers"] and point_in_poly(p["at"], z["outline"])]
        assert len(inside) >= 2, (name, len(inside))


def test_silkscreen_and_vias_clean():
    """No silkscreen warning (overlap, over a pad, off the edge) and no dangling via or track end."""
    r = json.loads(DRC.read_text())
    kinds = {}
    for v in r.get("violations", []):
        kinds[v["type"]] = kinds.get(v["type"], 0) + 1
    bad = {k: n for k, n in kinds.items() if k.startswith("silk") or k in ("via_dangling", "track_dangling")}
    assert not bad, bad


if __name__ == "__main__":
    lay_ = Layout(json.loads(DUMP.read_text()))
    isl = lay_.island()
    print("island", isl, "%.1f x %.1f mm" % (isl[2] - isl[0], isl[3] - isl[1]))
    got, dshot = imu_clearances(lay_)
    for imu, (a, b) in got.items():
        print(imu, "to J2 %.2f mm, to DShot copper %.2f mm" % (a, b))
    print("island edge to DShot copper %.2f mm (informative)" % min(lay_.dist(t, isl) for t in dshot))
    for net, (length, n) in dshot_lengths(lay_).items():
        print(net, "%.2f mm in %d segments" % (length, n))
    q = qspi_lengths(lay_)
    print("QSPI MCU-to-flash lengths:", ", ".join("%s %.2f" % (n[5:], v) for n, v in q.items()), "-> spread %.2f mm" % (max(q.values()) - min(q.values())))

    def net_len(net):
        return sum(t["len"] for t in lay_.tracks if t["net"] == net)
    print("USB lengths (informative): D+ %.1f mm, D- %.1f mm" % (net_len("USB_P"), net_len("USB_N")))
    print("track length on In2.Cu (L3): %.1f mm in %d segments" % (
        sum(t["len"] for t in lay_.tracks if t["layer"] == "In2.Cu"), sum(1 for t in lay_.tracks if t["layer"] == "In2.Cu")))
    widen.report(widen.stats(lay_.d))
    for z in lay_.d["zones"]:
        if z["name"] in POWER_POURS:
            print("%s (%s): %.1f mm2 filled" % (z["name"], z["net"], z["filled"]["B.Cu"]["area"]))
    raise SystemExit(pytest.main([os.path.abspath(__file__), "-q"]))

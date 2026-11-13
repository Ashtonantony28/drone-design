#!/usr/bin/env python3
"""Scripted pre-routing for nf_fc: everything that should not be left to the autorouter.

    python3 layout/preroute.py layout/build/board.json layout/build/preroute.json

Reads the geometry dump of the *placed, unrouted* board and writes tracks and
vias (mm, board-centre coordinates) that layout/route.py adds to the board as
locked copper before the Specctra export:

  1. MCU supply pin -> its decoupling capacitor, straight          (DESIGN.md 5.1)
  2. DShot: MCU pin -> via under the MCU -> 33R resistor on L4      (DESIGN.md 5.5)
     USB: D+/D- as a 90 ohm pair on L1, length-matched             (DESIGN.md 5.8)
     switch node: buck SW pad -> inductor; straight links between facing pads of one net
  3. one via beside every pad on a plane net (GND on L2; 3V3_D,
     3V3_IMU on L3 where their pour is under the pad)             (DESIGN.md 5.1/5.4)
  4. GND stitching vias along the board edges and around the buck  (DESIGN.md 5.4)

Pure Python (no pcbnew), deterministic: same placement in, same copper out.
"""
import json
import math
import sys

BOARD = 36.0
CORNER_R = 3.0
HOLE_XY = 15.25
HOLE_KEEPOUT_R = 3.5
EDGE = 0.35            # copper to board edge (rule: 0.3)
VIA_D, VIA_DRILL = 0.45, 0.3
CLR = 0.125            # copper clearance used here (rule: 0.1)
CLR_POWER = 0.175      # nets of the Power / USB classes (rule: 0.15)
HOLE_CLR = 0.25        # drilled hole to foreign copper (rule: 0.2)
VIA_PITCH = 0.60       # centre to centre: 0.45 pad + 0.1, and 0.3 drill + 0.25 hole-to-hole
PAD_GAP = 0.05         # via edge to an SMD pad of its own net: no via-in-pad
WIDE_NETS = {"5V", "VBUS", "VBAT", "PWR.IMU_LDO_IN", "USB_P", "USB_N"}
PLANES = {"GND": None, "3V3_D": "L3_3V3_D", "3V3_IMU": "L3_3V3_IMU"}
PLANE_MARGIN = 0.5     # via centre inside its L3 pour by at least this much
# Escape lanes: the strip that continues a fine-pitch pad (IC pin) along its
# axis is kept clear of other nets' vias and tracks, so that the scripted
# fan-out never walls a neighbouring signal pin in.
LANE_LEN = 0.6         # beyond each end of the pad
LANE_VIA = 0.43        # foreign via centre to lane centre line: leaves room for a 0.15 mm track
LANE_TRACK = 0.30      # foreign track centre line to lane centre line

MCU = "MCU.U1.X"
MCU_DECAPS = {
    "6": "MCU.C_VBAT_6.C", "11": "MCU.C_VDD_11.C", "21": "MCU.C_VDDA_100N.C", "20": "MCU.C_VDDA_100N.C",
    "27": "MCU.C_VDD_27.C", "50": "MCU.C_VDD_50.C", "75": "MCU.C_VDD_75.C", "100": "MCU.C_VDD_100.C",
    "48": "MCU.C_VCAP_48.C", "73": "MCU.C_VCAP_73.C",
}
DSHOT = {"MOTOR0": "CONN.R_M1.R", "MOTOR1": "CONN.R_M2.R", "MOTOR2": "CONN.R_M3.R", "MOTOR3": "CONN.R_M4.R"}
BUCK = "PWR.U6.X"
INDUCTOR = "PWR.L1.X"
SW_NET = "PWR.BUCK_SW"
L3_LINKS = [("VBAT", "PWR.C_VBAT3.C", "PWR.C_VBAT1.C", 0.5)]   # net, pad, pad, track width on In2.Cu
# Hand-drawn tracks where the geometry leaves one sensible path: (net, layer, width, points)
MANUAL = [
    # bootstrap: buck pin 4 down past the VCC pin, under the capacitor's SW pad, up into its BOOT pad
    ("PWR.BUCK_BOOT", "B", 0.15, [(9.6, 5.0), (9.6, 5.9), (9.95, 6.25), (11.3, 6.25), (11.54, 5.85)]),
    # crystal: straight from the MCU pins, between the decoupling capacitors, before any via goes there
    ("MCU.HSE_IN", "F", 0.15, [(2.5, -6.6), (2.45, -7.1), (2.45, -9.7)]),
    ("MCU.HSE_OUT", "F", 0.15, [(2.0, -6.6), (2.0, -7.2), (1.65, -7.6), (1.65, -11.4), (1.25, -11.9)]),
    # switch node to the bootstrap capacitor: out of pin 3, down between pin 4 and the input capacitor
    ("PWR.BUCK_SW", "B", 0.2, [(9.7, 4.475), (10.0, 4.475), (10.1, 4.6), (10.1, 5.0), (10.45, 5.45)]),
    # VBAT test point: straight up to the input capacitor, between TP9 and the feedback parts
    # (the USB pair crosses the top side here, so this connection has L4 only)
    ("VBAT", "B", 0.4, [(5.2, 9.6), (5.0, 9.3), (5.0, 5.45)]),
]
# USB D+/D-: a 90 ohm edge-coupled pair on L1 over the unbroken L2 ground.  Width and gap come
# from layout/impedance.py (field solution on the board's own stackup): 0.24 mm / 0.15 mm ->
# 89.8 ohm; the gap is drawn 5 um over the 0.15 mm clearance of the net class -> 90.3 ohm.
USB = {"P": "USB_P", "N": "USB_N", "width": 0.24, "gap": 0.155, "neck": 0.2,
       "esd": "CONN.U9.X", "conn": "CONN.J1.X",
       "level": 6.5,        # y of the pair's run under the MCU: the bottom pin row keeps a row of escape vias below it
       "knee": 0.6,         # size of the 45 degree corner from the pins into that run
       "leave": -5.0,       # x where the pair ends, short of the connector's vias
       "bump": 0.6, "bumps": 3, "bump_from": 4.9}   # length compensation on the shorter line
STITCH_PITCH = 3.0
STITCH_INSET = 0.9
# exposed pads that take vias in the pad: the DNP FPGA's ground pad (no other
# way out of a QFN) -- DESIGN.md 1 allows via-in-pad only for thermal pads
# The QFN sits between the board edge, a mounting hole and the buck: its pins
# are fanned out by the maze router, which can rip up and retry.
NO_FANOUT = {"FPGA.U10.X"}
# signal / rail pins of those packages that get their escape via scripted as well: the 1V2 pin of
# the FPGA sits under the MCU's pin row, between CDONE and the flash chip select
ESCAPE_VIAS = {("FPGA.U10.X", "5")}
FANOUT_MAX = 1.8       # longest pad-to-via track of the fan-out; a longer one fences its neighbours in
VIA_IN_PAD = {("FPGA.U10.X", "49"): [(-1.5, -1.5), (1.5, -1.5), (-1.5, 1.5), (1.5, 1.5)]}


def clr_for(a, b):
    return CLR_POWER if (a in WIDE_NETS or b in WIDE_NETS) else CLR


def point_rect(p, r):
    dx = max(r[0] - p[0], p[0] - r[2], 0.0)
    dy = max(r[1] - p[1], p[1] - r[3], 0.0)
    return math.hypot(dx, dy)


def point_seg(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2))
    return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)


def seg_points(a, b, step=0.04):
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]


def point_in_poly(p, poly):
    inside = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < (x2 - x1) * (p[1] - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def poly_edge_dist(p, poly):
    return min(point_seg(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))


class Router:
    def __init__(self, data):
        self.fp = {f["path"]: f for f in data["footprints"]}
        self.zones = {z["name"]: z for z in data["zones"]}
        self.pads = []  # copper pads and holes
        for f in data["footprints"]:
            for p in f["pads"]:
                if p["layers"] or p["drill"]:
                    q = dict(p)
                    q["path"] = f["path"]
                    q["ref"] = f["ref"]
                    q["through"] = p["drill"] > 0
                    self.pads.append(q)
        count = {}
        for p in self.pads:
            count[p["net"]] = count.get(p["net"], 0) + 1
        self.lanes = []  # (net, layers, a, b): centre line of the lane, end to end
        for p in self.pads:
            w, h = p["box"][2] - p["box"][0], p["box"][3] - p["box"][1]
            if p["through"] or not p["net"] or count[p["net"]] < 2 or min(w, h) > 0.35 or max(w, h) < 2 * min(w, h):
                continue
            x, y = p["at"]
            if w > h:
                a, b = (p["box"][0] - LANE_LEN, y), (p["box"][2] + LANE_LEN, y)
            else:
                a, b = (x, p["box"][1] - LANE_LEN), (x, p["box"][3] + LANE_LEN)
            self.lanes.append((p["net"], p["layers"], a, b))
        # rule areas that forbid tracks, per copper layer: [(layer, bbox)]
        self.no_track = []
        for z in data["zones"]:
            if z["keepout"] and z["name"].startswith("IMU_ISLAND"):
                xs, ys = [q[0] for q in z["outline"]], [q[1] for q in z["outline"]]
                for layer in z["layers"]:
                    self.no_track.append((layer[0], (min(xs), min(ys), max(xs), max(ys))))
        self.tracks = []
        self.vias = []
        self.log = []

    # ---- rules ------------------------------------------------------------
    def on_board(self, c, margin):
        h = BOARD / 2 - margin
        if abs(c[0]) > h or abs(c[1]) > h:
            return False
        k = BOARD / 2 - CORNER_R
        if abs(c[0]) > k and abs(c[1]) > k and math.hypot(abs(c[0]) - k, abs(c[1]) - k) > CORNER_R - margin:
            return False
        return math.hypot(abs(c[0]) - HOLE_XY, abs(c[1]) - HOLE_XY) >= HOLE_KEEPOUT_R + margin - EDGE + 0.05

    def plane_ok(self, c, net):
        zone = PLANES[net]
        if zone is None:
            return True
        poly = self.zones[zone]["outline"]
        return point_in_poly(c, poly) and poly_edge_dist(c, poly) >= PLANE_MARGIN

    def via_ok(self, c, net, dia=VIA_D, in_pad=None, lanes=True):
        r = dia / 2
        if not self.on_board(c, EDGE + r):
            return False
        for p in self.pads:
            if p is in_pad or (p["net"] == net and (p["path"], p["number"]) in VIA_IN_PAD):
                continue   # a thermal pad takes vias of its own net (DESIGN.md 1)
            d = point_rect(c, p["box"])
            if d > 1.2:
                continue
            if p["through"]:
                need = r + (HOLE_CLR if p["net"] != net else 0.15)
            elif p["net"] == net:
                need = r + PAD_GAP
            else:
                need = r + clr_for(net, p["net"])
            if d < need:
                return False
        for lnet, _, a, b in self.lanes if lanes else ():
            if lnet != net and abs(c[0] - a[0]) < 3 and abs(c[1] - a[1]) < 3 and point_seg(c, a, b) < LANE_VIA:
                return False
        for v in self.vias:
            if math.hypot(c[0] - v["at"][0], c[1] - v["at"][1]) < max(VIA_PITCH, r + v["dia"] / 2 + clr_for(net, v["net"])):
                return False
        for t in self.tracks:
            if t["net"] != net and point_seg(c, t["start"], t["end"]) < r + t["width"] / 2 + clr_for(net, t["net"]):
                return False
        return True

    def track_ok(self, a, b, width, layer, net, lanes=True):
        pts = seg_points(a, b)
        lo = (min(a[0], b[0]) - 1.0, min(a[1], b[1]) - 1.0, max(a[0], b[0]) + 1.0, max(a[1], b[1]) + 1.0)
        for q in pts:
            if not self.on_board(q, EDGE + width / 2):
                return False
        for lay, box in self.no_track:
            if lay == layer and any(point_rect(q, box) < width / 2 + 0.05 for q in pts):
                return False
        for lnet, llayers, la, lb in self.lanes if lanes else ():
            if lnet != net and layer in llayers and la[0] < lo[2] + 1 and lb[0] > lo[0] - 1 and la[1] < lo[3] + 1 and lb[1] > lo[1] - 1:
                if any(point_seg(q, la, lb) < LANE_TRACK for q in pts):
                    return False
        for p in self.pads:
            if p["net"] == net or not (p["through"] or layer in p["layers"]):
                continue
            bx = p["box"]
            if bx[2] < lo[0] or bx[0] > lo[2] or bx[3] < lo[1] or bx[1] > lo[3]:
                continue
            need = width / 2 + (HOLE_CLR if p["through"] else clr_for(net, p["net"]))
            if any(point_rect(q, bx) < need for q in pts):
                return False
        for v in self.vias:
            if v["net"] != net:
                need = width / 2 + v["dia"] / 2 + clr_for(net, v["net"])
                if any(math.hypot(q[0] - v["at"][0], q[1] - v["at"][1]) < need for q in pts):
                    return False
        for t in self.tracks:
            if t["net"] != net and t["layer"] == layer:
                need = width / 2 + t["width"] / 2 + clr_for(net, t["net"])
                if any(point_seg(q, t["start"], t["end"]) < need for q in pts):
                    return False
        return True

    # ---- output -----------------------------------------------------------
    def add_track(self, a, b, width, layer, net):
        if math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6:
            return
        self.tracks.append({"net": net, "layer": layer, "start": [round(a[0], 4), round(a[1], 4)],
                            "end": [round(b[0], 4), round(b[1], 4)], "width": width})

    def add_via(self, c, net, dia=VIA_D, drill=VIA_DRILL):
        self.vias.append({"net": net, "at": [round(c[0], 4), round(c[1], 4)], "dia": dia, "drill": drill})

    def pad(self, path, number=None, net=None):
        for p in self.pads:
            if p["path"] == path and (number is None or p["number"] == number) and (net is None or p["net"] == net):
                return p
        raise KeyError((path, number, net))

    # ---- 1. MCU pin -> decoupling capacitor --------------------------------
    def mcu_decaps(self):
        linked = set()
        for pin, cap in MCU_DECAPS.items():
            mp = self.pad(MCU, pin)
            cp = self.pad(cap, net=mp["net"])
            a, b = mp["at"], cp["at"]
            # leave the pin along its axis, then jog on to the capacitor pad
            vertical = (mp["box"][3] - mp["box"][1]) > (mp["box"][2] - mp["box"][0])
            if vertical:
                end = mp["box"][1] if b[1] < a[1] else mp["box"][3]
                knee = (a[0], end + (0.12 if b[1] > a[1] else -0.12))
            else:
                end = mp["box"][0] if b[0] < a[0] else mp["box"][2]
                knee = (end + (0.12 if b[0] > a[0] else -0.12), a[1])
            for w in (0.25, 0.2, 0.15):
                if self.track_ok(a, knee, w, "F", mp["net"], lanes=False) and self.track_ok(knee, b, w, "F", mp["net"], lanes=False):
                    self.add_track(a, knee, w, "F", mp["net"])
                    self.add_track(knee, b, w, "F", mp["net"])
                    linked.add((MCU, pin))
                    break
            else:
                self.log.append("decap link failed: pin %s -> %s" % (pin, cap))
        return linked

    # ---- 2. DShot escapes ---------------------------------------------------
    def dshot(self):
        cx = self.fp[MCU]["at"][0]
        for net, res in DSHOT.items():
            mp = self.pad(MCU, net=net)
            rp = self.pad(res, net=net)
            inner = mp["box"][2] if mp["at"][0] < cx else mp["box"][0]
            sign = 1 if mp["at"][0] < cx else -1
            best = None
            for i in range(0, 16):
                for j in range(-8, 9):
                    c = (inner + sign * (0.4 + 0.1 * i), mp["at"][1] + 0.1 * j)
                    knee = (inner + sign * 0.15, mp["at"][1])
                    if not self.via_ok(c, net):
                        continue
                    if not (self.track_ok(mp["at"], knee, 0.15, "F", net) and self.track_ok(knee, c, 0.15, "F", net)):
                        continue
                    if not self.track_ok(c, rp["at"], 0.15, "B", net):
                        continue
                    cost = math.dist(knee, c) + math.dist(c, rp["at"])
                    if best is None or cost < best[0]:
                        best = (cost, c, knee)
            if best is None:
                self.log.append("DShot escape failed: %s" % net)
                continue
            _, c, knee = best
            self.add_track(mp["at"], knee, 0.15, "F", net)
            self.add_track(knee, c, 0.15, "F", net)
            self.add_via(c, net)
            self.add_track(c, rp["at"], 0.15, "B", net)

    # ---- 2b. switch node: buck SW pad straight to the inductor (DESIGN.md 5.3) ----
    def switch_node(self):
        sw = [p for p in self.pads if p["path"] == BUCK and p["net"] == SW_NET]
        lp = self.pad(INDUCTOR, net=SW_NET)
        big = max(sw, key=lambda p: (p["box"][2] - p["box"][0]) * (p["box"][3] - p["box"][1]))
        lb = lp["box"]
        # from the end of the SW pad that faces the inductor to the nearest point of the inductor pad
        up = lp["at"][1] < big["at"][1]
        a = (big["at"][0], big["box"][1] + 0.2 if up else big["box"][3] - 0.2)
        b = (min(max(a[0], lb[0] + 0.25), lb[2] - 0.25), min(max(a[1], lb[1] + 0.25), lb[3] - 0.25))
        for w in (0.4, 0.3, 0.25):
            if self.track_ok(a, b, w, "B", SW_NET, lanes=False):
                self.add_track(a, b, w, "B", SW_NET)
                return
        self.log.append("switch-node link failed")

    # ---- 2e. USB: a coupled pair, MCU -> connector (DESIGN.md 5.8) -----------------
    def usb(self):
        """D+/D- as one pair on L1 from the MCU pins to the USB-C connector on L4.

        The pins are in the bottom row; the connector pads are fenced in on L4 by its
        shield pins and its own VBUS/GND pads, and on L1 the strip below the MCU is cut
        by a shield pin.  So the pair leaves the pins inwards, runs under the MCU along
        y = level, and leaves through the corner between the left and the bottom pin row
        (1.0 mm of room on the diagonal; the pair needs 0.94 mm).  At the connector the
        two contacts of each line (A6/B6, A7/B7) are joined by a link, one above the pad
        row and one below it, each with its via in the middle of the link, so both plug
        orientations see the same lengths.  The line that goes below the row has the
        longer way; the other gets the difference back as bumps in the straight run
        under the MCU.  The ESD array (place.py turns it so that no pin is fenced in)
        taps one line from its link and the other from a via beside the pair."""
        P, N, w, neck = USB["P"], USB["N"], USB["width"], USB["neck"]
        h = (w + USB["gap"]) / 2
        mp, mn = self.pad(MCU, net=P), self.pad(MCU, net=N)
        pads = {n: sorted((p for p in self.pads if p["path"] == USB["conn"] and p["net"] == n), key=lambda p: p["at"][0]) for n in (P, N)}
        esd = {n: sorted((p for p in self.pads if p["path"] == USB["esd"] and p["net"] == n), key=lambda p: -p["at"][1]) for n in (P, N)}
        if not (len(pads[P]) == len(pads[N]) == 2 and len(esd[P]) == len(esd[N]) == 2):
            self.log.append("USB pair: unexpected pads, left to the autorouter")
            return
        # the corner of the MCU the pair leaves through: between the last pad of the left row and the first of the bottom row
        mcu = [p for p in self.pads if p["path"] == MCU]
        left = max((p for p in mcu if p["box"][2] < mp["box"][0] and p["box"][2] - p["box"][0] > p["box"][3] - p["box"][1] and p["at"][0] < self.fp[MCU]["at"][0]), key=lambda p: p["at"][1])
        row = min((p for p in mcu if abs(p["at"][1] - mp["at"][1]) < 1e-6), key=lambda p: p["at"][0])
        diag = (left["box"][2] + row["box"][0]) / 2 + (left["box"][3] + row["box"][1]) / 2     # x + y on the centre line
        x0 = (mp["at"][0] + mn["at"][0]) / 2
        y_in = mp["box"][1] - 0.425
        y0, k = USB["level"], USB["knee"]
        centre = [(x0, y_in), (x0, y0 + k), (x0 - k, y0), (diag - y0, y0), (USB["leave"], diag - USB["leave"])]

        def offset(pts, d):
            out = []
            lines = []
            for a, b in zip(pts, pts[1:]):
                L = math.dist(a, b)
                ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
                lines.append(((a[0] + d * uy, a[1] - d * ux), (ux, uy)))
            out.append(lines[0][0])
            for (p1, u1), (p2, u2) in zip(lines, lines[1:]):
                den = u1[0] * u2[1] - u1[1] * u2[0]
                t = ((p2[0] - p1[0]) * u2[1] - (p2[1] - p1[1]) * u2[0]) / den
                out.append((p1[0] + t * u1[0], p1[1] + t * u1[1]))
            (p, u), a, b = lines[-1], pts[-2], pts[-1]
            out.append((p[0] + u[0] * math.dist(a, b), p[1] + u[1] * math.dist(a, b)))
            return out

        side = 1 if mn["at"][0] < mp["at"][0] else -1          # D- on the left of the direction of travel
        lines = {N: offset(centre, side * h), P: offset(centre, -side * h)}
        # "near" is the line on the connector's side of the diagonal: it ends at a via above the
        # pad row; "far" goes on past the row.  The ESD array has the near line's pins towards J1.
        near = min((P, N), key=lambda n: lines[n][-1][0] + lines[n][-1][1])
        far = N if near == P else P
        if min(p["at"][1] for p in esd[near]) < max(p["at"][1] for p in esd[far]):
            self.log.append("USB pair: the ESD array does not face the connector with %s, left to the autorouter" % near)
            return
        top = pads[near][0]["box"][1]
        yc = pads[near][0]["at"][1]
        y_n = top - 0.15 - VIA_D / 2 - 0.07
        y_s = yc + (yc - y_n)
        v_near = ((pads[near][0]["at"][0] + pads[near][1]["at"][0]) / 2, y_n)
        v_far = ((pads[far][0]["at"][0] + pads[far][1]["at"][0]) / 2, y_s)
        ln, lf = lines[near], lines[far]
        c_near, c_far = ln[-1][0] + ln[-1][1], lf[-1][0] + lf[-1][1]
        ln += [(v_near[0] + 0.15, c_near - v_near[0] - 0.15), v_near]
        x_f = v_far[0] + 0.5                 # the far line turns south here, clear of the near via
        lf += [(x_f, c_far - x_f), (x_f, y_s - 0.5), v_far]
        for n, m in ((P, mp), (N, mn)):
            lines[n][0:0] = [tuple(m["at"]), (m["at"][0], m["box"][1] - 0.175)]

        def length(pts):
            return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))

        # length compensation: rectangular bumps in the straight run of the shorter line, away from its partner
        short = min((ln, lf), key=length)
        need = abs(length(ln) - length(lf))
        i = next(i for i, (a, b) in enumerate(zip(short, short[1:])) if abs(a[1] - b[1]) < 1e-9 and abs(a[0] - b[0]) > 3)
        ya = short[i][1]
        away = 1 if ya > y0 else -1
        depth = need / (2 * USB["bumps"])
        bump = []
        for j in range(USB["bumps"]):
            xs = USB["bump_from"] - 2 * j * USB["bump"]
            bump += [(xs, ya), (xs, ya + away * depth), (xs - USB["bump"], ya + away * depth), (xs - USB["bump"], ya)]
        short[i + 1:i + 1] = bump
        skew = abs(length(ln) - length(lf))
        # L4: the two links at the connector, the links between the array's pin pairs, the taps
        b_tracks = []
        for net, v in ((near, v_near), (far, v_far)):
            for p in pads[net]:
                b_tracks.append((net, [tuple(p["at"]), (p["at"][0], v[1]), v], neck))
            a, b = sorted(esd[net], key=lambda p: p["at"][0])
            b_tracks.append((net, [(a["box"][2] - 0.15, a["at"][1]), (b["box"][0] + 0.15, b["at"][1])], neck))
        b_tracks.append((near, [v_near, (v_near[0], esd[near][0]["at"][1])], neck))
        # far line: a via beside it, on the side away from its partner, in the run under the MCU
        f_tracks = [(near, ln, w), (far, lf, w)]
        k_far = next(i for i, (a, b) in enumerate(zip(lf, lf[1:])) if abs(a[1] - b[1]) < 1e-9 and abs(a[0] - b[0]) > 3)
        y_far = lf[k_far][1]
        out = 1 if y_far > y0 else -1
        e = max(esd[far], key=lambda p: p["at"][0])
        vt = None
        for step in range(40):
            c = (e["box"][2] + 1.2 + 0.1 * step, y_far + out * 0.15)
            arm = [c, (c[0] - 0.45, e["at"][1] + 0.45), (e["box"][2] - 0.15, e["at"][1])]
            if c[0] < min(lf[k_far][0], lf[k_far + 1][0]) + 0.3:
                continue
            if self.via_ok(c, far) and all(self.track_ok(a, b, neck, "B", far, lanes=False) for a, b in zip(arm, arm[1:])):
                vt = c
                f_tracks.append((far, [c, (c[0], y_far)], neck))
                b_tracks.append((far, arm, neck))
                break
        # every piece is tested against the copper laid so far (not against its partner: the gap is by construction).
        # The two vias sit in the escape lane of the partner's pads on purpose: those pads are joined on the other side.
        ok = self.via_ok(v_near, near, lanes=False) and self.via_ok(v_far, far, lanes=False)
        for layer, group in (("F", f_tracks), ("B", b_tracks)):
            for net, pts, width in group:
                for a, b in zip(pts, pts[1:]):
                    if not self.track_ok(a, b, width, layer, net, lanes=False):
                        self.log.append("USB pair blocked: %s on %s from (%.2f, %.2f) to (%.2f, %.2f)" % (net, layer, a[0], a[1], b[0], b[1]))
                        ok = False
        if not ok:
            self.log.append("USB pair not routed: left to the autorouter")
            return
        for layer, group in (("F", f_tracks), ("B", b_tracks)):
            for net, pts, width in group:
                for a, b in zip(pts, pts[1:]):
                    self.add_track(a, b, width, layer, net)
        for net, v in ((near, v_near), (far, v_far)) + (((far, vt),) if vt else ()):
            self.add_via(v, net)
            self.vias[-1]["shared"] = False
        self.log.append("USB pair: %.2f mm on L1, %s %.3f mm, %s %.3f mm to the vias (skew %.3f mm), %d bumps of %.2f mm%s" % (
            length(centre), near, length(ln), far, length(lf), skew, USB["bumps"], depth, "" if vt else "; no room for the ESD via of " + far))

    def manual(self):
        for net, layer, width, pts in MANUAL:
            if all(self.track_ok(a, b, width, layer, net, lanes=False) for a, b in zip(pts, pts[1:])):
                for a, b in zip(pts, pts[1:]):
                    self.add_track(a, b, width, layer, net)
            else:
                self.log.append("manual track blocked: %s" % net)

    # ---- 2d. VBAT across the buck on L3 (DESIGN.md 1: VBAT on L3) ---------------
    def l3_links(self):
        """The buck has a VIN pin and capacitors on both sides, with the switch
        node between them: the two sides are joined on L3, straight under it."""
        for net, pa, pb, width in L3_LINKS:
            a, b = self.pad(pa, net=net), self.pad(pb, net=net)
            best = None
            for side in (1, -1):
                for i in range(6, 30):
                    y = (max(a["box"][3], b["box"][3]) + 0.05 * i) if side > 0 else (min(a["box"][1], b["box"][1]) - 0.05 * i)
                    for dxa in (0.0, 0.3, -0.3, 0.6, -0.6):
                        for dxb in (0.0, -0.3, 0.3, -0.6, 0.6):
                            ca, cb = (a["at"][0] + dxa, y), (b["at"][0] + dxb, y)
                            ea = (ca[0], a["box"][3] - 0.2 if side > 0 else a["box"][1] + 0.2)
                            eb = (cb[0], b["box"][3] - 0.2 if side > 0 else b["box"][1] + 0.2)
                            if not (self.via_ok(ca, net, dia=0.6) and self.via_ok(cb, net, dia=0.6)):
                                continue
                            if not (self.track_ok(ea, ca, 0.4, a["layers"][0], net) and self.track_ok(eb, cb, 0.4, b["layers"][0], net)):
                                continue
                            if not self.track_ok(ca, cb, width, "I", net):
                                continue
                            best = (ca, cb, ea, eb)
                            break
                        if best:
                            break
                    if best:
                        break
                if best:
                    break
            if not best:
                self.log.append("L3 link failed: %s %s - %s" % (net, pa, pb))
                continue
            ca, cb, ea, eb = best
            self.add_track(ea, ca, 0.4, a["layers"][0], net)
            self.add_track(eb, cb, 0.4, b["layers"][0], net)
            self.add_via(ca, net, dia=0.6)
            self.add_via(cb, net, dia=0.6)
            self.add_track(ca, cb, width, "I", net)

    # ---- 2c. neighbours: pads of one net that face each other get a straight link ----
    def short_links(self, reach=1.3):
        """A capacitor beside its pin, two pins of one rail on a small package:
        the straight track is the only sensible route, and left to the maze
        router the gap between the pads gets used by something else first."""
        by_net = {}
        for i, p in enumerate(self.pads):
            if p["net"] and p["net"] not in PLANES and not p["through"] and p["layers"]:
                by_net.setdefault(p["net"], []).append(i)
        made = 0
        for net, idx in sorted(by_net.items()):
            group = {i: i for i in idx}

            def find(i):
                while group[i] != i:
                    i = group[i]
                return i
            # copper already laid for this net joins the pads it touches
            for t in self.tracks:
                if t["net"] == net:
                    hit = [i for i in idx if t["layer"] in self.pads[i]["layers"] and
                           (point_rect(t["start"], self.pads[i]["box"]) < 1e-6 or point_rect(t["end"], self.pads[i]["box"]) < 1e-6)]
                    for i in hit[1:]:
                        group[find(i)] = find(hit[0])
            pairs = []
            for n, i in enumerate(idx):
                for j in idx[n + 1:]:
                    a, b = self.pads[i], self.pads[j]
                    if a["layers"] != b["layers"]:
                        continue
                    gap = math.hypot(max(a["box"][0] - b["box"][2], b["box"][0] - a["box"][2], 0.0),
                                     max(a["box"][1] - b["box"][3], b["box"][1] - a["box"][3], 0.0))
                    if gap <= reach:
                        pairs.append((gap, i, j))
            for gap, i, j in sorted(pairs):
                if find(i) == find(j):
                    continue
                a, b = self.pads[i], self.pads[j]
                layer = a["layers"][0]
                small = min(a["box"][2] - a["box"][0], a["box"][3] - a["box"][1], b["box"][2] - b["box"][0], b["box"][3] - b["box"][1])
                for w in ((0.25, 0.2, 0.15) if net in WIDE_NETS or net.startswith("PWR.") or net == SW_NET else (0.15,)):
                    if w > small + 1e-9:
                        continue
                    h = w / 2

                    def inside(p, q):   # the point of pad p nearest to q, half a track width inside the pad
                        bx = p["box"]
                        return (min(max(q[0], bx[0] + h), bx[2] - h), min(max(q[1], bx[1] + h), bx[3] - h))
                    s = inside(a, b["at"])
                    e = inside(b, s)
                    s = inside(a, e)
                    if self.track_ok(s, e, w, layer, net, lanes=False):
                        self.add_track(s, e, w, layer, net)
                        group[find(i)] = find(j)
                        made += 1
                        break
        return made

    # ---- 3. plane fan-out ---------------------------------------------------
    def fanout(self, skip):
        todo = [p for p in self.pads if (p["net"] in PLANES or (p["path"], p["number"]) in ESCAPE_VIAS)
                and not p["through"] and p["layers"] and (p["path"], p["number"]) not in skip]
        # crowded pads first: they have the fewest places for a via
        def crowd(p):
            return -sum(1 for q in self.pads if q is not p and point_rect(p["at"], q["box"]) < 1.2)
        # the packages that only fan out along their pins go first: they have one place each
        todo.sort(key=lambda p: (p["path"] not in NO_FANOUT, crowd(p), p["ref"], p["number"], p["at"]))
        offsets = sorted(((dx * 0.1, dy * 0.1) for dx in range(-24, 25) for dy in range(-24, 25)), key=lambda o: (round(math.hypot(*o), 3), o))
        orphans = []
        for p in todo:
            key = (p["path"], p["number"])
            if key in VIA_IN_PAD:
                for dx, dy in VIA_IN_PAD[key]:
                    c = (p["at"][0] + dx, p["at"][1] + dy)
                    if self.via_ok(c, p["net"], in_pad=p):
                        self.add_via(c, p["net"])
                continue
            layer = p["layers"][0]
            bx = p["box"]
            small = min(bx[2] - bx[0], bx[3] - bx[1])
            width = 0.3 if small >= 0.45 else 0.25 if small >= 0.28 else 0.2
            if p["net"] in WIDE_NETS and small >= 0.5:
                width = 0.4
            inset = min(width / 2, small / 2)
            done = False
            # share a via of the same net that is already next to the pad
            for v in sorted(self.vias, key=lambda v: point_rect(v["at"], bx)):
                d = point_rect(v["at"], bx)
                if d > 0.7:
                    break
                if v["net"] != p["net"] or not v.get("shared", True):
                    continue
                start = (min(max(v["at"][0], bx[0] + inset), bx[2] - inset), min(max(v["at"][1], bx[1] + inset), bx[3] - inset))
                if self.track_ok(start, v["at"], width, layer, p["net"]):
                    self.add_track(start, v["at"], width, layer, p["net"])
                    done = True
                    break
            if done:
                continue
            if p["path"] in NO_FANOUT:
                # fine-pitch package in a tight spot: only straight out along the pin, so that
                # no neighbour is walled in; whatever finds no place is left to the maze router
                cx, cy = self.fp[p["path"]]["at"]
                w, h = bx[2] - bx[0], bx[3] - bx[1]
                ux, uy = ((1 if p["at"][0] > cx else -1), 0) if w > h else (0, (1 if p["at"][1] > cy else -1))
                tip = (bx[2] if ux > 0 else bx[0] if ux < 0 else p["at"][0], bx[3] if uy > 0 else bx[1] if uy < 0 else p["at"][1])
                for i in range(6, 28):
                    c = (tip[0] + ux * 0.05 * i, tip[1] + uy * 0.05 * i)
                    if (p["net"] not in PLANES or self.plane_ok(c, p["net"])) and self.via_ok(c, p["net"]) \
                            and self.track_ok(p["at"], c, 0.2, layer, p["net"], lanes=False):
                        self.add_track(p["at"], c, 0.2, layer, p["net"])
                        self.add_via(c, p["net"])
                        break
                else:
                    orphans.append(p)
                continue
            for dx, dy in offsets:
                # walk outwards from the pad outline
                c = (min(max(p["at"][0] + dx * 3, bx[0]), bx[2]) + dx, min(max(p["at"][1] + dy * 3, bx[1]), bx[3]) + dy)
                if point_rect(c, bx) < VIA_D / 2 + PAD_GAP:
                    continue
                if (p["net"] in PLANES and not self.plane_ok(c, p["net"])) or not self.via_ok(c, p["net"]):
                    continue
                start = (min(max(c[0], bx[0] + inset), bx[2] - inset), min(max(c[1], bx[1] + inset), bx[3] - inset))
                if math.dist(start, c) > FANOUT_MAX or not self.track_ok(start, c, width, layer, p["net"]):
                    continue
                self.add_track(start, c, width, layer, p["net"])
                self.add_via(c, p["net"])
                done = True
                break
            if not done:
                orphans.append(p)

        # No place for a via: a straight track to the nearest via of the net, or to a pad that
        # reached one.  The MCU supply pins whose capacitor found no via are in the same position.
        for pin, cap in MCU_DECAPS.items():
            mp = self.pad(MCU, pin)
            if mp["net"] in PLANES and self.pad(cap, net=mp["net"]) in orphans:
                orphans.append(mp)
        progress = True
        while progress and orphans:
            progress = False
            for p in list(orphans):
                layer, bx = p["layers"][0], p["box"]
                small = min(bx[2] - bx[0], bx[3] - bx[1])
                cands = [(point_rect(v["at"], bx), tuple(v["at"])) for v in self.vias if v["net"] == p["net"]]
                cands += [(point_rect(q["at"], bx), tuple(q["at"])) for q in todo
                          if q is not p and q["net"] == p["net"] and q not in orphans and layer in q["layers"]]
                done = False
                for d, target in sorted(cands):
                    if d > 2.0:
                        break
                    for w in (0.25, 0.2, 0.15):
                        if w > small + 1e-9:
                            continue
                        h = w / 2
                        start = (min(max(target[0], bx[0] + h), bx[2] - h), min(max(target[1], bx[1] + h), bx[3] - h))
                        if self.track_ok(start, target, w, layer, p["net"], lanes=False):
                            self.add_track(start, target, w, layer, p["net"])
                            done = True
                            break
                    if done:
                        break
                if done:
                    orphans.remove(p)
                    progress = True
        for p in orphans:
            self.log.append("no plane via for %s.%s (%s) - left to the autorouter" % (p["ref"], p["number"], p["net"]))

    # ---- 4. stitching ---------------------------------------------------------
    def stitch(self):
        placed = 0
        lim = BOARD / 2 - STITCH_INSET
        span = HOLE_XY - HOLE_KEEPOUT_R - 0.6
        n = int(2 * span / STITCH_PITCH)
        nominal = [-span + i * (2 * span / n) for i in range(n + 1)]
        shifts = sorted((s * 0.15 for s in range(-9, 10)), key=abs)
        for axis in (0, 1):
            for sign in (-1, 1):
                for u in nominal:
                    hit = False
                    for inset in (0.0, -0.3, 0.2, 0.4, 0.6):
                        for s in shifts:
                            c = [0.0, 0.0]
                            c[axis] = sign * (lim - inset)
                            c[1 - axis] = u + s
                            if self.via_ok(tuple(c), "GND"):
                                self.add_via(tuple(c), "GND")
                                self.vias[-1]["stitch"] = True
                                placed += 1
                                hit = True
                                break
                        if hit:
                            break
                    if not hit:
                        self.log.append("no edge stitching via near %s=%.1f on the %s%s edge" % ("xy"[1 - axis], u, "-+"[sign > 0], "xy"[axis]))
        # close the gaps the nominal positions left (connectors and test pads at the edge)
        span_lim = HOLE_XY - HOLE_KEEPOUT_R
        for axis in (0, 1):
            for sign in (-1, 1):
                for _ in range(6):
                    along = sorted(v["at"][1 - axis] for v in self.vias if v["net"] == "GND" and abs(v["at"][axis] - sign * BOARD / 2) <= 1.5
                                   and abs(v["at"][1 - axis]) <= span_lim)
                    pts = [-span_lim] + along + [span_lim]
                    gap = max(zip(pts, pts[1:]), key=lambda ab: ab[1] - ab[0])
                    if gap[1] - gap[0] <= STITCH_PITCH + 0.5:
                        break
                    mid = (gap[0] + gap[1]) / 2
                    cands = []
                    for i in range(int((gap[1] - gap[0] - 1.0) / 0.1) + 1):
                        u = gap[0] + 0.5 + 0.1 * i
                        for inset in (-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6):
                            c = [0.0, 0.0]
                            c[axis] = sign * (lim - inset)
                            c[1 - axis] = u
                            cands.append((abs(u - mid), abs(inset), tuple(c)))
                    for _, _, c in sorted(cands):
                        if self.via_ok(c, "GND"):
                            self.add_via(c, "GND")
                            self.vias[-1]["stitch"] = True
                            placed += 1
                            break
                    else:
                        self.log.append("edge stitching gap of %.1f mm on the %s%s edge stays" % (gap[1] - gap[0], "-+"[sign > 0], "xy"[axis]))
                        break
        # ring around the buck converter
        cy = self.fp[BUCK]["courtyard"]
        ring = (cy[0] - 0.5, cy[1] - 0.5, cy[2] + 0.5, cy[3] + 0.5)
        per = []
        steps = 8
        ly = self.fp[INDUCTOR]["at"][1]
        for i in range(steps):
            f = (i + 0.5) / steps
            # three sides only: the side facing the inductor stays open for the switch node
            side_to_l1 = ring[1] if abs(ring[1] - ly) < abs(ring[3] - ly) else ring[3]
            per += [(ring[0] + f * (ring[2] - ring[0]), ring[3] if side_to_l1 == ring[1] else ring[1]),
                    (ring[0], ring[1] + f * (ring[3] - ring[1])), (ring[2], ring[1] + f * (ring[3] - ring[1]))]
        near = 0
        for u in per:
            for dx, dy in sorted(((a * 0.15, b * 0.15) for a in range(-7, 8) for b in range(-7, 8)), key=lambda o: math.hypot(*o)):
                c = (u[0] + dx, u[1] + dy)
                if self.via_ok(c, "GND"):
                    self.add_via(c, "GND")
                    self.vias[-1]["stitch"] = True
                    near += 1
                    break
        return placed, near


def main(src, dst):
    data = json.load(open(src))
    assert not data["tracks"] and not data["vias"], "pre-route starts from the placed, unrouted board (run place first)"
    r = Router(data)
    linked = r.mcu_decaps()
    r.dshot()
    r.switch_node()
    r.usb()
    r.l3_links()
    r.manual()
    links = r.short_links()
    r.fanout(skip=linked)
    n_fan = len(r.vias)
    edge, buck = r.stitch()
    json.dump({"tracks": r.tracks, "vias": r.vias}, open(dst, "w"), indent=0)
    print("pre-route: %d tracks, %d vias (%d fan-out/DShot, %d edge stitching, %d around the buck), %d neighbour links" % (len(r.tracks), len(r.vias), n_fan, edge, buck, links))
    for line in r.log:
        print("  note:", line)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])

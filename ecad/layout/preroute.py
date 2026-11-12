#!/usr/bin/env python3
"""Scripted pre-routing for nf_fc: everything that should not be left to the autorouter.

    python3 layout/preroute.py layout/build/board.json layout/build/preroute.json

Reads the geometry dump of the *placed, unrouted* board and writes tracks and
vias (mm, board-centre coordinates) that layout/route.py adds to the board as
locked copper before the Specctra export:

  1. MCU supply pin -> its decoupling capacitor, straight          (DESIGN.md 5.1)
  2. DShot: MCU pin -> via under the MCU -> 33R resistor on L4      (DESIGN.md 5.5)
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
STITCH_PITCH = 3.0
STITCH_INSET = 0.9
# exposed pads that take vias in the pad: the DNP FPGA's ground pad (no other
# way out of a QFN) -- DESIGN.md 1 allows via-in-pad only for thermal pads
# The QFN sits between the board edge, a mounting hole and the buck: its pins
# are fanned out by the maze router, which can rip up and retry.
NO_FANOUT = {"FPGA.U10.X"}
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

    def via_ok(self, c, net, dia=VIA_D, in_pad=None):
        r = dia / 2
        if not self.on_board(c, EDGE + r):
            return False
        for p in self.pads:
            if p is in_pad:
                continue
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
        for lnet, _, a, b in self.lanes:
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

    # ---- 3. plane fan-out ---------------------------------------------------
    def fanout(self, skip):
        todo = [p for p in self.pads if p["net"] in PLANES and not p["through"] and p["layers"]
                and (p["path"], p["number"]) not in skip
                and not (p["path"] in NO_FANOUT and (p["path"], p["number"]) not in VIA_IN_PAD)]
        # crowded pads first: they have the fewest places for a via
        def crowd(p):
            return -sum(1 for q in self.pads if q is not p and point_rect(p["at"], q["box"]) < 1.2)
        todo.sort(key=lambda p: (crowd(p), p["ref"], p["number"], p["at"]))
        offsets = sorted(((dx * 0.1, dy * 0.1) for dx in range(-24, 25) for dy in range(-24, 25)), key=lambda o: (round(math.hypot(*o), 3), o))
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
            for dx, dy in offsets:
                # walk outwards from the pad outline
                c = (min(max(p["at"][0] + dx * 3, bx[0]), bx[2]) + dx, min(max(p["at"][1] + dy * 3, bx[1]), bx[3]) + dy)
                if point_rect(c, bx) < VIA_D / 2 + PAD_GAP:
                    continue
                if not self.plane_ok(c, p["net"]) or not self.via_ok(c, p["net"]):
                    continue
                start = (min(max(c[0], bx[0] + inset), bx[2] - inset), min(max(c[1], bx[1] + inset), bx[3] - inset))
                if not self.track_ok(start, c, width, layer, p["net"]):
                    continue
                self.add_track(start, c, width, layer, p["net"])
                self.add_via(c, p["net"])
                done = True
                break
            if not done:
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
        # ring around the buck converter
        cy = self.fp[BUCK]["courtyard"]
        ring = (cy[0] - 0.5, cy[1] - 0.5, cy[2] + 0.5, cy[3] + 0.5)
        per = []
        steps = 6
        ly = self.fp[INDUCTOR]["at"][1]
        for i in range(steps):
            f = (i + 0.5) / steps
            # three sides only: the side facing the inductor stays open for the switch node
            side_to_l1 = ring[1] if abs(ring[1] - ly) < abs(ring[3] - ly) else ring[3]
            per += [(ring[0] + f * (ring[2] - ring[0]), ring[3] if side_to_l1 == ring[1] else ring[1]),
                    (ring[0], ring[1] + f * (ring[3] - ring[1])), (ring[2], ring[1] + f * (ring[3] - ring[1]))]
        near = 0
        for u in per:
            for dx, dy in sorted(((a * 0.15, b * 0.15) for a in range(-5, 6) for b in range(-5, 6)), key=lambda o: math.hypot(*o)):
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
    r.fanout(skip=linked)
    n_fan = len(r.vias)
    edge, buck = r.stitch()
    json.dump({"tracks": r.tracks, "vias": r.vias}, open(dst, "w"), indent=0)
    print("pre-route: %d tracks, %d vias (%d fan-out/DShot, %d edge stitching, %d around the buck)" % (len(r.tracks), len(r.vias), n_fan, edge, buck))
    for line in r.log:
        print("  note:", line)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])

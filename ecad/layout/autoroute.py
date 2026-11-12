#!/usr/bin/env python3
"""Grid maze router for nf_fc: routes everything preroute.py left open.

    python3 layout/autoroute.py layout/build/board.json layout/build/autoroute.json

Input is the geometry dump of the placed and pre-routed board; output is a
list of tracks and vias (mm, board-centre coordinates) for route.py to add.

Method: A* on a 0.1 mm grid over the two outer layers (L2 and L3 stay planes),
8-direction moves plus vias, one connection at a time, short nets first.  A
connection that cannot be completed is retried at the end by ripping up the
routed nets in its way.  Clearances are baked into occupancy maps (copper
inflated by clearance + half the track width), so a finished route is clean
by construction; KiCad's DRC stays the judge (layout/run.sh drc).

Pure Python + numpy, deterministic.
"""
import heapq
import json
import math
import sys
import time

import numpy as np

GRID = 0.1
BOARD = 36.0
N = int(round(BOARD / GRID)) + 1
CORNER_R = 3.0
HOLE_XY = 15.25
HOLE_KEEPOUT_R = 3.5
EDGE = 0.35
CLR = 0.125                 # copper clearance used (rule 0.1)
CLR_POWER = 0.175           # Power / USB classes (rule 0.15)
HOLE_CLR = 0.25             # hole to foreign copper (rule 0.2)
TRACK = 0.15
WIDE = 0.4
VIA_D, VIA_DRILL = 0.45, 0.3
VIA_PITCH = 0.6
VIA_PAD_GAP = 0.05          # via edge to any SMD pad: no via-in-pad
POWER_CLASS = {"5V", "VBUS", "VBAT", "PWR.IMU_LDO_IN", "USB_P", "USB_N"}
WIDE_NETS = {"VBAT", "PWR.BUCK_SW"}   # 0.4 mm (they also get an L4 pour); they neck down at their pads
WIDTHS = {"USB_P": 0.19, "USB_N": 0.19, "5V": 0.25, "VBUS": 0.25, "PWR.IMU_LDO_IN": 0.25, "VBAT": 0.25, "PWR.BUCK_SW": 0.25}
NARROW_POWER = 0.25         # widest track the Q map is good for
PLANES = {"GND": None, "3V3_D": "L3_3V3_D", "3V3_IMU": "L3_3V3_IMU"}
PLANE_MARGIN = 0.5
ISLAND_RING = 1.0
ISLAND_NETS = ("3V3_IMU", "GND", "IMUA_", "IMUB_", "IMU.")
# routed first, in this order (DESIGN.md 5.5-5.8), then everything else by size
FIRST = ["ESC_M1", "CONN.ESC_M2", "CONN.ESC_M3", "CONN.ESC_M4", "USB_P", "USB_N",
         "PWR.BUCK_SW", "PWR.BUCK_BOOT", "PWR.BUCK_FB", "PWR.BUCK_VCC", "VBAT",
         "IMUA_SCK", "IMUA_MOSI", "IMUA_MISO", "IMUA_CS", "IMU.IMUA_SCK_R",
         "IMUB_SCK", "IMUB_MOSI", "IMUB_MISO", "IMUB_CS", "IMU.IMUB_SCK_R",
         "MCU.HSE_IN", "MCU.HSE_OUT", "VDDA"]
ORTH, DIAG, VIA_COST = 10, 14, 300
PROX_COST = 6               # per step within PROX_DIST of a pad: keeps tracks from fencing pins in
PROX_DIST = 0.45
# Escape lanes: the strip that continues an IC pin along its axis.  Other nets
# pay to run through it and may not put a via in it, so pins do not get fenced in.
LANE_LEN = 1.5
LANE_HALF = 0.2
LANE_COST = 40              # per step in a foreign lane
LANE_VIA_LEN = 1.0
LANE_VIA_HALF = 0.43
PRES_START = 10             # price of a cell inside another routed net's halo, first iteration
PRES_GROWTH = 1.3           # ... multiplied each iteration
PRES_MAX = 1500
HIST_FRACTION = 0.25        # a cell that stays contested gets this share of the price added for good
MAX_ITERATIONS = 120
PATIENCE = 30               # stop when the best state is this many iterations old
LAYERS = ("F", "I", "B")    # I = In2.Cu (L3); L2 stays an unbroken ground plane
INNER_COST = 12             # per step on L3: outer layers first, L3 "where needed" (DESIGN.md 1)
# nets that stay on the outer layers: fast edges, the buck, and the planes themselves
OUTER_ONLY = ("USB_", "QSPI_", "IMUA_", "IMUB_", "IMU.", "MOTOR", "ESC_M", "CONN.ESC_M", "MCU.HSE", "PWR.BUCK", "VBAT",
              "GND", "3V3_D", "3V3_IMU", "VDDA")


def is_island_net(net):
    return any(net == p or (p.endswith(("_", ".")) and net.startswith(p)) for p in ISLAND_NETS)


def cell(v):
    return int(round((v + BOARD / 2) / GRID))


def coord(i):
    return round(i * GRID - BOARD / 2, 4)


class Maps:
    """Occupancy maps.  Each holds, per cell, 0 (free), a net id (inside the
    keep-off halo of that net's copper only) or -1 (blocked for everyone)."""

    def __init__(self):
        ax = np.arange(N) * GRID - BOARD / 2
        self.X, self.Y = np.meshgrid(ax, ax)
        # keep-off maps for: a standard track centre (T), a narrow track of a
        # power-class net (Q), a wide track or a power-class via (W), a via (V)
        self.T = {L: np.zeros((N, N), np.int32) for L in LAYERS}
        self.Q = {L: np.zeros((N, N), np.int32) for L in LAYERS}
        self.W = {L: np.zeros((N, N), np.int32) for L in LAYERS}
        self.V = {L: np.zeros((N, N), np.int32) for L in LAYERS}
        # cells where a track of that width lies wholly on a pad
        self.deep = {L: np.zeros((N, N), bool) for L in LAYERS}
        self.deep_wide = {L: np.zeros((N, N), bool) for L in LAYERS}
        self.viaban = np.zeros((N, N), bool)
        self.comp = {L: np.zeros((N, N), np.int32) for L in LAYERS}

    def window(self, box, r):
        x0, y0 = max(0, cell(box[0] - r) - 1), max(0, cell(box[1] - r) - 1)
        x1, y1 = min(N, cell(box[2] + r) + 2), min(N, cell(box[3] + r) + 2)
        return slice(y0, y1), slice(x0, x1)

    def dist_rect(self, win, box):
        X, Y = self.X[win], self.Y[win]
        dx = np.maximum(np.maximum(box[0] - X, X - box[2]), 0)
        dy = np.maximum(np.maximum(box[1] - Y, Y - box[3]), 0)
        return np.hypot(dx, dy)

    def dist_seg(self, win, a, b):
        X, Y = self.X[win], self.Y[win]
        vx, vy = b[0] - a[0], b[1] - a[1]
        L2 = vx * vx + vy * vy
        if L2 == 0:
            return np.hypot(X - a[0], Y - a[1])
        t = np.clip(((X - a[0]) * vx + (Y - a[1]) * vy) / L2, 0, 1)
        return np.hypot(X - a[0] - t * vx, Y - a[1] - t * vy)

    @staticmethod
    def mark(arr, win, mask, nid):
        sub = arr[win]
        sub[mask & (sub != 0) & (sub != nid)] = -1
        sub[mask & (sub == 0)] = nid


class Router:
    def __init__(self, data):
        self.data = data
        self.m = Maps()
        self.net_id = {}
        self.net_name = {}
        self.zones = {z["name"]: z for z in data["zones"]}
        self.parent = [0]          # union-find over copper components
        self.comp_net = [0]
        self.anchor = {}           # (layer, y, x) -> exact point of pre-routed copper
        self.plane_comp = {}
        self.out_tracks = []
        self.out_vias = []
        self.routes = {}           # net -> list of (tracks, vias) added by this router
        self.failed = []
        self.keepout_b = None
        for z in data["zones"]:
            if z["keepout"] and z["name"].startswith("IMU_ISLAND"):
                xs, ys = [q[0] for q in z["outline"]], [q[1] for q in z["outline"]]
                self.keepout_b = (min(xs), min(ys), max(xs), max(ys))
        self.island = self.keepout_b   # the L4 keep-out is drawn on the island rectangle
        self.fake = 0
        self._near = {}
        self.prox = {L: np.zeros((N, N), np.uint8) for L in LAYERS}
        self.lane = {L: np.zeros((N, N), np.int32) for L in LAYERS}      # net id owning the lane cell, -1 if shared
        self.lane_via = np.zeros((N, N), np.int32)
        self.build()

    # ---- nets and components ----------------------------------------------
    def nid(self, net):
        if net not in self.net_id:
            self.net_id[net] = len(self.net_id) + 1
            self.net_name[self.net_id[net]] = net
        return self.net_id[net]

    def new_comp(self, net):
        self.parent.append(len(self.parent))
        self.comp_net.append(self.nid(net))
        return len(self.parent) - 1

    def find(self, c):
        while self.parent[c] != c:
            self.parent[c] = self.parent[self.parent[c]]
            c = self.parent[c]
        return c

    def union(self, a, b):
        a, b = self.find(a), self.find(b)
        if a != b:
            self.parent[max(a, b)] = min(a, b)

    def set_comp(self, layer, win, mask, c):
        sub = self.m.comp[layer][win]
        for other in np.unique(sub[mask]):
            if other and self.comp_net[other] == self.comp_net[c]:
                self.union(c, int(other))
        sub[mask] = c

    # ---- stamping copper ----------------------------------------------------
    def clr(self, net):
        return CLR_POWER if net in POWER_CLASS else CLR

    def halo(self, layer, kind, geom, net, extra=0.0):
        """Keep-off halo of a piece of copper as (map name, window, mask) for the
        four maps: T standard track centre, Q narrow power-class track centre,
        W wide track or power-class via centre, V via centre."""
        c = (HOLE_CLR if net is None else self.clr(net)) + extra
        m = self.m
        cp = max(c, CLR_POWER + extra)
        for name, r in (("T", c + TRACK / 2), ("Q", cp + NARROW_POWER / 2), ("W", cp + max(WIDE, VIA_D) / 2), ("V", c + VIA_D / 2)):
            if kind == "rect":
                win = m.window(geom, r)
                d = m.dist_rect(win, geom)
            elif kind == "seg":
                a, b, w = geom
                win = m.window((min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])), r + w / 2)
                d = m.dist_seg(win, a, b) - w / 2
            else:
                p, dia = geom
                win = m.window((p[0], p[1], p[0], p[1]), r + dia / 2)
                d = m.dist_seg(win, p, p) - dia / 2
            yield name, win, d < r - 1e-9

    def stamp_halo(self, layer, kind, geom, net, extra=0.0):
        nid = -1 if net is None else self.nid(net)
        maps = {"T": self.m.T, "Q": self.m.Q, "W": self.m.W, "V": self.m.V}
        for name, win, mask in self.halo(layer, kind, geom, net, extra):
            self.m.mark(maps[name][layer], win, mask, nid)

    def add_pad(self, p):
        layers = LAYERS if p["drill"] > 0 else [L for L in ("F", "B") if L in p["layers"]]
        box = p["box"]
        if p["npth"] or not p["net"]:
            for L in LAYERS if p["drill"] > 0 else layers:
                self.fake += 1
                self.stamp_halo(L, "rect", box, None if p["drill"] > 0 else "~nc%d" % self.fake)
        else:
            for L in layers:
                self.stamp_halo(L, "rect", box, p["net"])
        # no via in or against a pad (any net); holes keep their distance from everything
        r = VIA_D / 2 + (HOLE_CLR if p["drill"] > 0 else VIA_PAD_GAP)
        win = self.m.window(box, r)
        self.m.viaban[win] |= self.m.dist_rect(win, box) < r
        for L in layers:
            win = self.m.window(box, PROX_DIST)
            self.prox[L][win][self.m.dist_rect(win, box) < PROX_DIST] = PROX_COST
        if not p["net"] or p["npth"]:
            return None
        c = self.new_comp(p["net"])
        for L in layers:
            inner = (box[0] + 0.05, box[1] + 0.05, box[2] - 0.05, box[3] - 0.05)
            win = self.m.window(box, 0)
            mask = self.m.dist_rect(win, inner) < 1e-9
            if not mask.any():
                d = np.hypot(self.m.X[win] - p["at"][0], self.m.Y[win] - p["at"][1])
                mask = d <= d.min() + 1e-9
            self.set_comp(L, win, mask, c)
            for arr, half in ((self.m.deep[L], TRACK / 2), (self.m.deep_wide[L], WIDE / 2)):
                if box[2] - box[0] >= 2 * half and box[3] - box[1] >= 2 * half:
                    arr[win] |= self.m.dist_rect(win, (box[0] + half, box[1] + half, box[2] - half, box[3] - half)) < 1e-9
        if p["drill"] > 0 and p["net"] in PLANES:
            self.union(c, self.plane(p["net"]))
        return c

    def plane(self, net):
        if net not in self.plane_comp:
            self.plane_comp[net] = self.new_comp(net)
        return self.plane_comp[net]

    def in_plane(self, p, net):
        zone = PLANES[net]
        if zone is None:
            return True
        poly = self.zones[zone]["outline"]
        inside = False
        for i in range(len(poly)):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
            if (y1 > p[1]) != (y2 > p[1]) and p[0] < (x2 - x1) * (p[1] - y1) / (y2 - y1) + x1:
                inside = not inside
        return inside

    def add_track(self, t, exact=True):
        L = t["layer"][0]
        a, b, w = t["start"], t["end"], t["width"]
        self.stamp_halo(L, "seg", (a, b, w), t["net"])
        c = self.new_comp(t["net"])
        win = self.m.window((min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])), 0.1)
        d = self.m.dist_seg(win, a, b)
        mask = d <= 0.0708
        self.set_comp(L, win, mask, c)
        if exact:
            ys, xs = np.nonzero(mask)
            vx, vy = b[0] - a[0], b[1] - a[1]
            L2 = vx * vx + vy * vy
            for yy, xx in zip(ys, xs):
                gy, gx = yy + win[0].start, xx + win[1].start
                px, py = coord(gx), coord(gy)
                tt = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - a[0]) * vx + (py - a[1]) * vy) / L2))
                self.anchor[(L, gy, gx)] = (round(a[0] + tt * vx, 4), round(a[1] + tt * vy, 4))
        return c

    def add_via(self, v, exact=True):
        p = v["at"]
        c = self.new_comp(v["net"])
        for L in LAYERS:
            self.stamp_halo(L, "via", (p, v["dia"]), v["net"], extra=0.02)  # keeps tracks 0.2 mm off the drill
            win = self.m.window((p[0], p[1], p[0], p[1]), 0.1)
            d = np.hypot(self.m.X[win] - p[0], self.m.Y[win] - p[1])
            mask = d <= max(0.0708, d.min() + 1e-9)
            self.set_comp(L, win, mask, c)
            if exact:
                ys, xs = np.nonzero(mask)
                for yy, xx in zip(ys, xs):
                    self.anchor[(L, yy + win[0].start, xx + win[1].start)] = (p[0], p[1])
        win = self.m.window((p[0], p[1], p[0], p[1]), VIA_PITCH)
        self.m.viaban[win] |= np.hypot(self.m.X[win] - p[0], self.m.Y[win] - p[1]) < VIA_PITCH - 1e-9
        if v["net"] in PLANES and self.in_plane(p, v["net"]):
            self.union(c, self.plane(v["net"]))
        return c

    def build(self):
        m = self.m
        # board outline, corner radii and mounting-hole keep-outs
        k = BOARD / 2 - CORNER_R
        ax, ay = np.abs(m.X), np.abs(m.Y)
        corner = (ax > k) & (ay > k)
        hole = np.hypot(ax - HOLE_XY, ay - HOLE_XY)
        for arrs, half in ((m.T, TRACK / 2), (m.Q, NARROW_POWER / 2), (m.W, WIDE / 2), (m.V, VIA_D / 2)):
            margin = EDGE + half
            h = BOARD / 2 - margin
            bad = (ax > h + 1e-9) | (ay > h + 1e-9) | (corner & (np.hypot(ax - k, ay - k) > CORNER_R - margin)) | (hole < HOLE_KEEPOUT_R + half + 0.05)
            for L in LAYERS:
                arrs[L][bad] = -1
        # L3 under and around the IMU island and its feed carries nothing but the 3V3_IMU pour
        o = self.zones["L3_3V3_IMU"]["outline"]
        xs, ys = sorted({q[0] for q in o}), sorted({q[1] for q in o})
        self.inner_block = np.zeros((N, N), bool)
        for q in o:
            pass
        i = self.keepout_b
        feed = (i[2], min(ys), max(xs), max(y for y in ys if y < i[3]))
        for box in (i, feed):
            win = m.window(box, ISLAND_RING + 0.3)
            self.inner_block[win] |= m.dist_rect(win, box) < ISLAND_RING + 0.2
        if self.keepout_b:
            win = m.window(self.keepout_b, 0.3)
            for arrs, half in ((m.T, TRACK / 2), (m.Q, NARROW_POWER / 2), (m.W, WIDE / 2)):
                arrs["B"][win][m.dist_rect(win, self.keepout_b) < half + 0.05] = -1
        for f in self.data["footprints"]:
            for p in f["pads"]:
                if p["layers"] or p["drill"]:
                    self.add_pad(p)
        count = {}
        for f in self.data["footprints"]:
            for p in f["pads"]:
                count[p["net"]] = count.get(p["net"], 0) + 1
        for f in self.data["footprints"]:
            for p in f["pads"]:
                b = p["box"]
                w, h = b[2] - b[0], b[3] - b[1]
                if p["drill"] or not p["layers"] or not p["net"] or count[p["net"]] < 2 or min(w, h) > 0.35 or max(w, h) < 2 * min(w, h):
                    continue
                nid = self.nid(p["net"])
                x, y = p["at"]
                for length, half, arrs in ((LANE_LEN, LANE_HALF, [self.lane[L] for L in ("F", "B") if L in p["layers"]]),
                                           (LANE_VIA_LEN, LANE_VIA_HALF, [self.lane_via])):
                    if w > h:
                        boxes = [(b[0] - length, y - half, b[0], y + half), (b[2], y - half, b[2] + length, y + half)]
                    else:
                        boxes = [(x - half, b[1] - length, x + half, b[1]), (x - half, b[3], x + half, b[3] + length)]
                    for box in boxes:
                        win = m.window(box, 0)
                        mask = m.dist_rect(win, box) < 1e-9
                        for arr in arrs:
                            m.mark(arr, win, mask, nid)
        for t in self.data["tracks"]:
            self.add_track(t)
        for v in self.data["vias"]:
            self.add_via(v)

    # ---- searching ------------------------------------------------------------
    def free_maps(self, net):
        """Cells a track centre of this net may use on each layer, and via cells,
        with respect to the fixed copper (pads, holes, edge, scripted pre-route)."""
        nid = self.nid(net)
        m = self.m
        special = net in POWER_CLASS or net in WIDE_NETS
        narrow = m.Q if special else m.T
        ring = None
        if not is_island_net(net):
            i = self.island
            ring = m.window((i[0] - ISLAND_RING, i[1] - ISLAND_RING, i[2] + ISLAND_RING, i[3] + ISLAND_RING), 0.3)
        free = {}
        near = self.near_pads(net) if net in WIDE_NETS else None
        for L in LAYERS:
            f = (narrow[L] == 0) | (narrow[L] == nid)
            if net in WIDE_NETS:
                # a wide net may neck down within 1 mm of its own pads
                f = (m.W[L] == 0) | (m.W[L] == nid) | (f & near)
            if ring:
                f[ring] = False
            if L == "I":
                if net.startswith(OUTER_ONLY):
                    f = np.zeros((N, N), bool)
                else:
                    f &= ~self.inner_block
            free[L] = f
        vmap = m.W if special else m.V
        via = ~m.viaban
        for L in LAYERS:
            via &= (vmap[L] == 0) | (vmap[L] == nid)
        if ring:
            via[ring] = False
        via &= (self.lane_via == 0) | (self.lane_via == nid)
        return free, via

    def near_pads(self, net):
        if net not in self._near:
            near = np.zeros((N, N), bool)
            for f in self.data["footprints"]:
                for p in f["pads"]:
                    if p["net"] == net:
                        win = self.m.window(p["box"], 1.0)
                        near[win] |= self.m.dist_rect(win, p["box"]) < 1.0
            self._near[net] = near
        return self._near[net]

    def components(self, net):
        nid = self.nid(net)
        roots = {}
        for c in range(1, len(self.parent)):
            if self.comp_net[c] == nid:
                roots.setdefault(self.find(c), []).append(c)
        return roots


def astar(sources, tgt, fr, xc, vi, vx, tb):
    """Cheapest path from any source cell to any target cell.  Cells are flat
    indexes layer*N*N + y*N + x; fr = passable, xc = extra cost of entering a
    cell, vi = via allowed (per x,y), vx = extra cost of a via there."""
    NN = N * N
    nl = len(LAYERS)
    tx0, ty0, tx1, ty1 = tb
    best, prev, heap = {}, {}, []
    push, pop = heapq.heappush, heapq.heappop
    for i in sources:
        best[i] = 0
        r = i % NN
        x, y = r % N, r // N
        dx = tx0 - x if x < tx0 else x - tx1 if x > tx1 else 0
        dy = ty0 - y if y < ty0 else y - ty1 if y > ty1 else 0
        push(heap, (ORTH * max(dx, dy) + (DIAG - ORTH) * min(dx, dy), 0, i))
    moves = ((1, ORTH), (-1, ORTH), (N, ORTH), (-N, ORTH), (N + 1, DIAG), (N - 1, DIAG), (-N + 1, DIAG), (-N - 1, DIAG))
    others = [[b for b in range(nl) if b != a] for a in range(nl)]
    INF = 1 << 60
    limit = 3000000
    while heap:
        f, g, i = pop(heap)
        if g > best.get(i, INF):
            continue
        if tgt[i]:
            path = [i]
            while i in prev:
                i = prev[i]
                path.append(i)
            return path[::-1]
        limit -= 1
        if limit < 0:
            return None
        li, r = divmod(i, NN)
        x, y = r % N, r // N
        for d, c in moves:
            rj = r + d
            if rj < 0 or rj >= NN:
                continue
            j = i + d
            xj = rj % N
            if xj - x > 1 or x - xj > 1 or not fr[j]:
                continue
            ng = g + c + xc[j]
            if ng < best.get(j, INF):
                best[j] = ng
                prev[j] = i
                yj = rj // N
                dx = tx0 - xj if xj < tx0 else xj - tx1 if xj > tx1 else 0
                dy = ty0 - yj if yj < ty0 else yj - ty1 if yj > ty1 else 0
                push(heap, (ng + ORTH * max(dx, dy) + (DIAG - ORTH) * min(dx, dy), ng, j))
        if vi[r]:
            for lj in others[li]:
                j = lj * NN + r
                if fr[j]:
                    ng = g + VIA_COST + xc[j] + vx[r]
                    if ng < best.get(j, INF):
                        best[j] = ng
                        prev[j] = i
                        dx = tx0 - x if x < tx0 else x - tx1 if x > tx1 else 0
                        dy = ty0 - y if y < ty0 else y - ty1 if y > ty1 else 0
                        push(heap, (ng + ORTH * max(dx, dy) + (DIAG - ORTH) * min(dx, dy), ng, j))
    return None


def _seg_dist(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2))
    return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)


class Negotiator:
    """Negotiated-congestion routing (PathFinder): every net is routed through
    the fixed obstacles as if alone, paying a price for cells inside another
    routed net's keep-off halo.  The price rises each iteration, and cells that
    stay contested collect a history cost, until no net touches another."""

    def __init__(self, data):
        self.data = data
        self.fx = Router(data)
        fx = self.fx
        self.cnt = {k: {L: np.zeros((N, N), np.int16) for L in LAYERS} for k in "TQWV"}
        self.vp = np.zeros((N, N), np.int16)       # via-to-via pitch discs of routed vias
        self.hist = {L: np.zeros((N, N), np.int32) for L in LAYERS}
        self.routes = {}
        nets = sorted({p["net"] for f in data["footprints"] for p in f["pads"] if p["net"]})
        self.nets = [n for n in nets if len(fx.components(n)) > 1]
        self.own = {}
        self.static = {}

    # ---- per-net constants ----------------------------------------------------
    def prepare(self, net):
        if net in self.static:
            return self.static[net]
        fx = self.fx
        nid = fx.nid(net)
        NN = N * N
        free, via = fx.free_maps(net)
        roots = fx.components(net)
        all_ids = [c for ids in roots.values() for c in ids]
        passable, lane_cost = [], []
        for L in LAYERS:
            own = np.isin(fx.m.comp[L], all_ids) & fx.m.deep[L]
            passable.append(free[L] | own)
            cost = fx.prox[L].astype(np.int32) + (INNER_COST if L == "I" else 0)
            cost = cost + np.where((fx.lane[L] != 0) & (fx.lane[L] != nid), LANE_COST, 0)
            lane_cost.append(cost)
        comps = []
        plane_root = fx.find(fx.plane_comp[net]) if net in fx.plane_comp else None
        for root, ids in sorted(roots.items()):
            cells = set()
            for li, L in enumerate(LAYERS):
                flat = np.flatnonzero(np.isin(fx.m.comp[L], ids) & passable[li])
                cells.update((flat + li * NN).tolist())
            comps.append((root == plane_root, cells))
        special = net in POWER_CLASS or net in WIDE_NETS
        wide_ok = None
        if net in WIDE_NETS:
            wide_ok = [(fx.m.W[L] == 0) | (fx.m.W[L] == nid) for L in LAYERS]
        st = {"passable": passable, "fr": b"".join(a.astype(np.uint8).tobytes() for a in passable), "via": via,
              "base": lane_cost, "comps": comps, "special": special, "wide_ok": wide_ok, "nid": nid}
        self.static[net] = st
        return st

    def class_count(self, net, st, li):
        L = LAYERS[li]
        if st["wide_ok"] is not None:
            return np.where(st["wide_ok"][li], self.cnt["W"][L], self.cnt["Q"][L])
        return self.cnt["Q" if st["special"] else "T"][L]

    def via_count(self, st):
        k = "W" if st["special"] else "V"
        total = self.vp.astype(np.int32)
        for L in LAYERS:
            total = total + self.cnt[k][L]
        return total

    # ---- one net ----------------------------------------------------------------
    def route(self, net, pres, hard=False, keep=()):
        """Route every open connection of a net; returns the number left open.
        keep: connections of this net (from an earlier call) to build on."""
        st = self.prepare(net)
        NN = N * N
        nl = len(LAYERS)
        counts = [self.class_count(net, st, li) for li in range(nl)]
        vcount = self.via_count(st)
        if hard:
            fr = b"".join((st["passable"][li] & (counts[li] == 0)).astype(np.uint8).tobytes() for li in range(nl))
            xc = np.concatenate([(st["base"][li] + self.hist[LAYERS[li]]).ravel() for li in range(nl)]).tolist()
            via = st["via"] & (vcount == 0)
            vx = [0] * NN
        else:
            fr = st["fr"]
            xc = np.concatenate([(st["base"][li] + self.hist[LAYERS[li]] + pres * np.minimum(counts[li], 3)).ravel()
                                 for li in range(nl)]).tolist()
            via = st["via"]
            vx = (pres * np.minimum(vcount, 3)).ravel().tolist()
        own_ban = np.zeros((N, N), bool)

        def ban(q):
            win = self.fx.m.window((q[0], q[1], q[0], q[1]), VIA_PITCH)
            own_ban[win] |= np.hypot(self.fx.m.X[win] - q[0], self.fx.m.Y[win] - q[1]) < VIA_PITCH - 1e-9

        comps = [(is_plane, set(cells)) for is_plane, cells in st["comps"]]
        conns = []
        for c in keep:
            path = c["cells"]
            ia = next((j for j, (_, cells) in enumerate(comps) if path[0] in cells), None)
            ib = next((j for j, (_, cells) in enumerate(comps) if path[-1] in cells), None)
            idx = sorted({j for j in (ia, ib) if j is not None})
            merged = (any(comps[j][0] for j in idx), set(path).union(*[comps[j][1] for j in idx]))
            comps = [c2 for j, c2 in enumerate(comps) if j not in idx] + [merged]
            conns.append(c)
            for q in c["vias"]:
                ban(q["at"])
        left = 0
        while len(comps) > 1:
            done = False
            for k in sorted((k for k in range(len(comps)) if not comps[k][0]), key=lambda k: (len(comps[k][1]), k)):
                src = comps[k][1]
                tgt = bytearray(nl * NN)
                for j, (_, cells) in enumerate(comps):
                    if j != k:
                        for i in cells:
                            tgt[i] = 1
                allc = np.fromiter((i for j, (_, cells) in enumerate(comps) if j != k for i in cells), dtype=np.int64)
                if len(allc) == 0 or not src:
                    continue
                rr = allc % NN
                tb = (int((rr % N).min()), int((rr // N).min()), int((rr % N).max()), int((rr // N).max()))
                vi = bytearray((via & ~own_ban).astype(np.uint8).tobytes())
                path = None
                for _ in range(6):
                    path = astar(src, tgt, fr, xc, vi, vx, tb)
                    if not path:
                        break
                    # two layer changes of one path closer than the via pitch: forbid the second and search again
                    vcells = [path[i] % NN for i in range(1, len(path)) if path[i] // NN != path[i - 1] // NN]
                    clash = [b for i, b in enumerate(vcells) for a in vcells[:i]
                             if math.hypot((a % N - b % N), (a // N - b // N)) * GRID < VIA_PITCH]
                    if not clash:
                        break
                    for b in clash:
                        vi[b] = 0
                    path = None
                if not path:
                    continue
                t, v = self.geometry(net, st, path)
                conns.append({"tracks": t, "vias": v, "cells": path})
                for q in v:
                    ban(q["at"])
                end = path[-1]
                j = next(j for j, (_, cells) in enumerate(comps) if j != k and end in cells)
                merged = (comps[k][0] or comps[j][0], comps[k][1] | comps[j][1] | set(path))
                comps = [c for idx, c in enumerate(comps) if idx not in (j, k)] + [merged]
                done = True
                break
            if not done:
                left = len(comps) - 1
                break
        self.routes[net] = {"conns": conns, "stamps": [], "open": left}
        self.stamp(net)
        return left

    def geometry(self, net, st, path):
        NN = N * N
        narrow_w = WIDTHS.get(net, TRACK)
        pts = []
        for i in path:
            li, r = divmod(i, NN)
            y, x = r // N, r % N
            w = WIDE if (st["wide_ok"] is not None and st["wide_ok"][li][y, x]) else narrow_w
            pts.append((LAYERS[li], y, x, w))
        tracks, vias = [], []

        def xy(p):
            return [coord(p[2]), coord(p[1])]

        def emit(a, b, w):
            if a[:3] != b[:3]:
                tracks.append({"net": net, "layer": a[0], "start": xy(a), "end": xy(b), "width": w})

        # exact joints to pre-routed copper at both ends
        for end in (pts[0], pts[-1]):
            a = self.fx.anchor.get(end[:3])
            if a and (abs(a[0] - xy(end)[0]) > 1e-6 or abs(a[1] - xy(end)[1]) > 1e-6):
                tracks.append({"net": net, "layer": end[0], "start": list(a), "end": xy(end), "width": narrow_w})
        run_start = prev_p = pts[0]
        direction = run_w = None
        for p in pts[1:]:
            if p[0] != prev_p[0]:
                emit(run_start, prev_p, run_w or narrow_w)
                vias.append({"net": net, "at": xy(p), "dia": VIA_D, "drill": VIA_DRILL})
                run_start, direction, run_w = p, None, None
            else:
                d = (p[1] - prev_p[1], p[2] - prev_p[2])
                seg_w = min(p[3], prev_p[3])   # a step is wide only if both of its cells allow it
                if direction is not None and (d != direction or seg_w != run_w):
                    emit(run_start, prev_p, run_w)
                    run_start = prev_p
                direction, run_w = d, seg_w
            prev_p = p
        emit(run_start, prev_p, run_w or narrow_w)
        return tracks, vias

    # ---- occupancy of routed nets ---------------------------------------------
    def stamp(self, net, sign=1):
        r = self.routes[net]
        if sign > 0 and not r["stamps"]:
            fx = self.fx
            for t in (t for c in r["conns"] for t in c["tracks"]):
                L = t["layer"][0]
                for k, win, mask in fx.halo(L, "seg", (t["start"], t["end"], t["width"]), net):
                    r["stamps"].append((self.cnt[k][L], win, mask))
            for v in (v for c in r["conns"] for v in c["vias"]):
                p = v["at"]
                for L in LAYERS:
                    for k, win, mask in fx.halo(L, "via", (p, v["dia"]), net, extra=0.02):
                        r["stamps"].append((self.cnt[k][L], win, mask))
                win = fx.m.window((p[0], p[1], p[0], p[1]), VIA_PITCH)
                r["stamps"].append((self.vp, win, np.hypot(fx.m.X[win] - p[0], fx.m.Y[win] - p[1]) < VIA_PITCH - 1e-9))
        for arr, win, mask in r["stamps"]:
            arr[win][mask] += sign

    def conflicts(self, net, per_conn=False):
        """Cells of a net's route that lie in another routed net's halo."""
        r = self.routes[net]
        if not r["conns"]:
            return []
        st = self.prepare(net)
        NN = N * N
        self.stamp(net, -1)
        counts = [self.class_count(net, st, li) for li in range(len(LAYERS))]
        vcount = self.via_count(st)
        out = []
        for c in r["conns"]:
            bad = []
            prev_li = None
            for i in c["cells"]:
                li, rr = divmod(i, NN)
                y, x = rr // N, rr % N
                if counts[li][y, x] > 0 or (prev_li is not None and prev_li != li and vcount[y, x] > 0):
                    bad.append((LAYERS[li], y, x))
                prev_li = li
            out.append(bad)
        self.stamp(net, 1)
        return out if per_conn else [q for bad in out for q in bad]

    def report_conflicts(self, order):
        """Say which nets fight over which spot (for tuning the placement)."""
        seen = {}
        for n in order:
            for L, y, x in self.conflicts(n):
                px, py = coord(x), coord(y)
                for m in order:
                    if m == n:
                        continue
                    r = self.routes[m]
                    hit = any(t["layer"][0] == L and abs(t["start"][0] - px) < 6 and
                              _seg_dist((px, py), t["start"], t["end"]) < 0.5 + t["width"] / 2 for c in r["conns"] for t in c["tracks"]) or \
                        any(math.hypot(v["at"][0] - px, v["at"][1] - py) < 0.9 for c in r["conns"] for v in c["vias"])
                    if hit:
                        key = tuple(sorted((n, m)))
                        seen.setdefault(key, (L, px, py))
        for (a, b), (L, px, py) in sorted(seen.items(), key=lambda kv: (kv[1][1], kv[1][2])):
            print("  conflict %s x %s near %s (%.1f, %.1f)" % (a, b, L, px, py))

    def snapshot(self):
        return {n: {"conns": list(r["conns"]), "open": r["open"]} for n, r in self.routes.items()}

    def restore(self, snap):
        for n in list(self.routes):
            self.stamp(n, -1)
        self.routes = {n: dict(r, stamps=[]) for n, r in snap.items()}
        for n in self.routes:
            self.stamp(n)

    # ---- driver ---------------------------------------------------------------------
    def run(self):
        data = self.data

        def size(n):
            pts = [p["at"] for f in data["footprints"] for p in f["pads"] if p["net"] == n]
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            return (max(xs) - min(xs)) + (max(ys) - min(ys))

        order = [n for n in FIRST if n in self.nets] + sorted((n for n in self.nets if n not in FIRST), key=lambda n: (n in PLANES, size(n), n))
        t0 = time.time()
        pres = PRES_START
        todo = list(order)
        best = None
        for it in range(MAX_ITERATIONS):
            for n in todo:
                if n in self.routes:
                    self.stamp(n, -1)
                self.route(n, pres)
            conf = {n: self.conflicts(n) for n in order}
            step = max(2, int(pres * HIST_FRACTION))
            for cells in conf.values():
                for L, y, x in cells:
                    self.hist[L][y, x] += step
            bad = [n for n in order if conf[n]]
            opened = [n for n in order if self.routes[n]["open"]]
            total = sum(len(c) for c in conf.values())
            print("iteration %d: %d nets rerouted, %d in conflict (%d cells), %d with open connections, %.0f s" % (
                it + 1, len(todo), len(bad), total, len(opened), time.time() - t0), flush=True)
            score = (len(opened), total)
            if best is None or score < best[0]:
                best = (score, it, self.snapshot())
            if not bad and not opened:
                break
            if it - best[1] >= PATIENCE:
                break
            todo = [n for n in order if conf[n] or self.routes[n]["open"]]
            pres = min(int(pres * PRES_GROWTH) + 1, PRES_MAX)
        self.restore(best[2])
        self.report_conflicts(order)
        # Legalise: take out the connection with the most overlap until nothing
        # overlaps, then put those connections back strictly around everything else.
        ripped = set()
        while True:
            worst = None
            for n in order:
                for k, bad in enumerate(self.conflicts(n, per_conn=True)):
                    key = (len(bad), -len(self.routes[n]["conns"][k]["cells"]))
                    if bad and (worst is None or key > worst[0]):
                        worst = (key, n, k)
            if worst is None:
                break
            _, n, k = worst
            self.stamp(n, -1)
            r = self.routes[n]
            self.routes[n] = {"conns": r["conns"][:k] + r["conns"][k + 1:], "stamps": [], "open": r["open"] + 1}
            self.stamp(n)
            ripped.add(n)
        print("legalise: connections taken out of %d nets (%s)" % (len(ripped), ", ".join(sorted(ripped))), flush=True)
        for attempt in range(3):
            for n in sorted((n for n in order if self.routes[n]["open"]), key=lambda n: (size(n), n)):
                self.stamp(n, -1)
                self.route(n, 0, hard=True, keep=self.routes[n]["conns"])
            if not any(self.routes[n]["open"] for n in order):
                break
        left = {n: self.routes[n]["open"] for n in order if self.routes[n]["open"]}
        tracks = [t for n in order for c in self.routes[n]["conns"] for t in c["tracks"]]
        vias = [v for n in order for c in self.routes[n]["conns"] for v in c["vias"]]
        return tracks, vias, left


def main(src, dst):
    data = json.load(open(src))
    r = Negotiator(data)
    tracks, vias, left = r.run()
    json.dump({"tracks": tracks, "vias": vias}, open(dst, "w"), indent=0)
    print("autoroute: %d tracks, %d vias, %d connections left open" % (len(tracks), len(vias), sum(left.values())))
    for n, k in sorted(left.items()):
        print("  open:", n, k)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])

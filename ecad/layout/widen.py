#!/usr/bin/env python3
"""Give the maze router's tracks margin over the 0.1 mm minimum wherever the layout has room.

    python3 layout/widen.py layout/build/pre.json layout/build/autoroute.json layout/build/routed.json
    python3 layout/widen.py --stats layout/build/board.json

The router works on a 0.1 mm grid with 0.1 mm tracks and 0.1 mm spaces (DESIGN.md 1: the fab's
minimum).  JLC's standard capability is 0.127 mm (5 mil); 0.1 mm is the limit.  This pass
widens the routed segments in place, never moving one, so connectivity cannot change:

  * power nets (POWER_LADDER) go up the ladder 0.5 ... 0.127 mm as far as the room allows,
  * every other 0.1 mm segment goes to 0.127 mm, and in a second round to 0.15 mm,

each time only if the space to all foreign copper on its layer (pads, tracks, vias, as they
are at that moment) stays at 0.127 mm or more (0.15 mm against Power / USB class nets, their
rule; 0.25 mm to an unplated hole), and the track stays clear of the board edge and of the
rule areas.  A segment that cannot be widened keeps its width.  KiCad's DRC still judges the
result.

stats() reports what is left: the share of track length narrower than 0.127 mm, the share of
track length that has foreign copper nearer than 0.127 mm, and where on the board that is.

Pure Python.
"""
import json
import math
import sys

SIGNAL_ROUNDS = (0.127, 0.15)
POWER_LADDER = (0.5, 0.4, 0.3, 0.25, 0.2, 0.15, 0.127)
# nets that carry supply current: as wide as the room allows (check_layout.py has the currents)
POWER_NETS = {"5V", "VBUS", "VBAT", "PWR.IMU_LDO_IN", "PWR.BUCK_SW", "3V3_D", "3V3_IMU", "VDDA", "1V2_FPGA", "GND"}
FROZEN = {"USB_P", "USB_N"}     # the 90 ohm pair keeps its calculated width
SPACE = 0.127
POWER_CLASS = {"5V", "VBUS", "VBAT", "PWR.IMU_LDO_IN", "USB_P", "USB_N"}
POWER_SPACE = 0.15
VIA_SPACE = 0.13       # keeps the 0.2 mm hole clearance to the via's drill (annular ring 0.075 mm)
HOLE_SPACE = 0.25      # unplated holes
BOARD = 36.0
CORNER_R = 3.0
EDGE = 0.32            # copper to board edge (rule 0.3)
AREA_SPACE = 0.03      # track edge to a rule area that forbids tracks
CELL = 1.0
EPS = 0.002
LAYERS = {"F": "F.Cu", "B": "B.Cu", "I": "In2.Cu"}


def seg_seg(a, b, c, d):
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    d1, d2, d3, d4 = cross(c, d, a), cross(c, d, b), cross(a, b, c), cross(a, b, d)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 * d2 != 0 and d3 * d4 != 0:
        return 0.0
    return min(pt_seg(a, c, d), pt_seg(b, c, d), pt_seg(c, a, b), pt_seg(d, a, b))


def pt_seg(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2))
    return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)


def seg_rect(a, b, r):
    if r[0] <= a[0] <= r[2] and r[1] <= a[1] <= r[3]:
        return 0.0
    c = [(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])]
    return min(seg_seg(a, b, c[i], c[(i + 1) % 4]) for i in range(4))


class Copper:
    """Every piece of copper per layer, in a coarse grid for neighbour look-up."""

    def __init__(self, data, extra_tracks=(), extra_vias=()):
        self.items = []      # [kind, net, layer, geometry, half-width or radius, source track]
        self.grid = {}
        self.areas = []      # (layer, outline) of the rule areas that forbid tracks
        for z in data.get("zones", ()):
            if z["keepout"]:
                for layer in z["layers"]:
                    self.areas.append((layer, [tuple(q) for q in z["outline"]]))
        for f in data["footprints"]:
            for p in f["pads"]:
                if not (p["layers"] or p["drill"]):
                    continue
                layers = list(LAYERS.values()) if p["drill"] else [LAYERS[x] for x in p["layers"]]
                hole = p["drill"] and (p["npth"] or not p["net"])
                for layer in layers:
                    self.add("hole" if hole else "pad", p["net"] or "~%s.%s" % (f["ref"], p["number"]), layer, tuple(p["box"]), 0.0)
        for t in list(data["tracks"]) + list(extra_tracks):
            self.add("track", t["net"], LAYERS.get(t["layer"], t["layer"]), (tuple(t["start"]), tuple(t["end"])), t["width"] / 2, t)
        for v in list(data["vias"]) + list(extra_vias):
            for layer in LAYERS.values():
                self.add("via", v["net"], layer, tuple(v["at"]), v["dia"] / 2)

    def add(self, kind, net, layer, geom, half, ref=None):
        if kind in ("pad", "hole"):
            box = geom
        elif kind == "track":
            (ax, ay), (bx, by) = geom
            box = (min(ax, bx) - half, min(ay, by) - half, max(ax, bx) + half, max(ay, by) + half)
        else:
            box = (geom[0] - half, geom[1] - half, geom[0] + half, geom[1] + half)
        idx = len(self.items)
        self.items.append([kind, net, layer, geom, half, ref])
        for gx in range(int(math.floor(box[0] / CELL)), int(math.floor(box[2] / CELL)) + 1):
            for gy in range(int(math.floor(box[1] / CELL)), int(math.floor(box[3] / CELL)) + 1):
                self.grid.setdefault((layer, gx, gy), []).append(idx)

    def near(self, layer, a, b, reach):
        seen = set()
        x0, x1 = min(a[0], b[0]) - reach, max(a[0], b[0]) + reach
        y0, y1 = min(a[1], b[1]) - reach, max(a[1], b[1]) + reach
        for gx in range(int(math.floor(x0 / CELL)), int(math.floor(x1 / CELL)) + 1):
            for gy in range(int(math.floor(y0 / CELL)), int(math.floor(y1 / CELL)) + 1):
                for idx in self.grid.get((layer, gx, gy), ()):
                    if idx not in seen:
                        seen.add(idx)
                        yield self.items[idx]

    @staticmethod
    def gap(a, b, kind, geom, ohalf):
        """Distance from the centre line a-b to the edge of another piece of copper."""
        if kind in ("pad", "hole"):
            return seg_rect(a, b, geom)
        if kind == "track":
            return seg_seg(a, b, geom[0], geom[1]) - ohalf
        return pt_seg(geom, a, b) - ohalf

    def space(self, net, layer, a, b, half, limit=1.0):
        """Smallest gap from the segment (a, b, half-width) to foreign copper, and that copper's net."""
        best, who = limit, None
        for kind, onet, _, geom, ohalf, _ in self.near(layer, a, b, limit + half + 0.5):
            if onet == net:
                continue
            d = self.gap(a, b, kind, geom, ohalf) - half
            if d < best:
                best, who = d, onet
        return best, who

    def fits(self, net, layer, a, b, w):
        """True if the segment may be w wide: spaces, board edge, rule areas."""
        h = w / 2
        lim = BOARD / 2 - EDGE - h
        k = BOARD / 2 - CORNER_R
        for p in (a, b):
            ax, ay = abs(p[0]), abs(p[1])
            if ax > lim or ay > lim or (ax > k and ay > k and math.hypot(ax - k, ay - k) > CORNER_R - EDGE - h):
                return False
        for alayer, poly in self.areas:
            if alayer == layer and min(seg_seg(a, b, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))) < h + AREA_SPACE:
                return False
        for kind, onet, _, geom, ohalf, _ in self.near(layer, a, b, h + HOLE_SPACE + 0.5):
            if onet == net:
                continue
            if self.gap(a, b, kind, geom, ohalf) - h < need(net, onet, kind) + EPS:
                return False
        return True


def need(net, other, kind="track"):
    if kind == "hole":
        return HOLE_SPACE
    if net in POWER_CLASS or other in POWER_CLASS:
        return POWER_SPACE
    return VIA_SPACE if kind == "via" else SPACE


PIECE = 0.1            # a segment that does not fit as a whole is tested in pieces of this length ...
MIN_RUN = 0.4          # ... and widened where at least this much of it fits (the rest stays as a neck)


def widen(pre, routed):
    cu = Copper(pre, routed["tracks"], routed["vias"])
    mine = {id(t) for t in routed["tracks"]}
    items = [it for it in cu.items if it[0] == "track" and id(it[5]) in mine and it[1] not in FROZEN]
    count = {}
    extra = []

    def grow(item, widths):
        _, net, layer, (a, b), _, t = item
        for w in widths:
            if w <= t["width"] + 1e-9:
                return True
            if cu.fits(net, layer, a, b, w):
                t["width"] = w
                item[4] = w / 2
                count[w] = count.get(w, 0) + 1
                return True
        return False

    def grow_part(item, w):
        """Widen the stretches of a segment that have room; the rest stays as it is (a neck)."""
        _, net, layer, (a, b), half, t = item
        L = math.dist(a, b)
        n = int(L / PIECE)
        if w <= t["width"] + 1e-9 or n < 2:
            return
        pts = [(round(a[0] + (b[0] - a[0]) * i / n, 4), round(a[1] + (b[1] - a[1]) * i / n, 4)) for i in range(n + 1)]
        pts[0], pts[-1] = a, b
        ok = [cu.fits(net, layer, pts[i], pts[i + 1], w) for i in range(n)]
        runs, i = [], 0
        while i < n:
            j = i
            while j < n and ok[j] == ok[i]:
                j += 1
            runs.append([i, j, ok[i]])
            i = j
        for r in runs:                       # short wide stretches are not worth two more joints
            if r[2] and (r[1] - r[0]) * L / n < MIN_RUN:
                r[2] = False
        merged = []
        for r in runs:
            if merged and merged[-1][2] == r[2]:
                merged[-1][1] = r[1]
            else:
                merged.append(list(r))
        if not any(r[2] for r in merged):
            return
        first = True
        for i, j, wide in merged:
            width = w if wide else t["width"]
            if first:
                piece, it = t, item
                first = False
                old_width = t["width"]
                t["end"] = list(pts[j])
                it[3] = (pts[i], pts[j])
            else:
                piece = dict(t, start=list(pts[i]), end=list(pts[j]), width=old_width)
                extra.append(piece)
                cu.add("track", net, layer, (pts[i], pts[j]), old_width / 2, piece)
                it = cu.items[-1]
                items.append(it)
            if wide:
                piece["width"] = width
                it[4] = width / 2
                count[w] = count.get(w, 0) + 1

    queue = [it for it in items if it[1] in POWER_NETS]
    while queue:
        item = queue.pop(0)
        if grow(item, POWER_LADDER):
            continue
        for w in POWER_LADDER:
            known = len(items)
            grow_part(item, w)
            if len(items) > known:           # split: every piece goes up the ladder on its own
                queue.extend([item] + items[known:])
                break
    for w in SIGNAL_ROUNDS:
        for item in list(items):
            if item[1] not in POWER_NETS and not grow(item, (w,)):
                grow_part(item, w)
    routed["tracks"].extend(extra)
    return count


def stats(data, step=0.1, region=6.0):
    """Track length by width, the share of it with foreign copper nearer than 0.127 mm, and
    where the tight copper is: mm of track per region x region mm square and layer."""
    cu = Copper(data)
    by_width = {}
    where = {}
    total = tight = narrow = 0.0
    for t in data["tracks"]:
        a, b = tuple(t["start"]), tuple(t["end"])
        L = math.dist(a, b)
        if L < 1e-9:
            continue
        by_width[round(t["width"], 3)] = by_width.get(round(t["width"], 3), 0.0) + L
        layer = LAYERS.get(t["layer"], t["layer"])
        n = max(1, int(round(L / step)))
        thin = t["width"] < SPACE - 1e-6
        for i in range(n):
            p = (a[0] + (b[0] - a[0]) * (i + 0.5) / n, a[1] + (b[1] - a[1]) * (i + 0.5) / n)
            d, _ = cu.space(t["net"], layer, p, p, t["width"] / 2, limit=0.3)
            total += L / n
            close = d < SPACE - EPS
            tight += L / n if close else 0.0
            narrow += L / n if thin else 0.0
            if close or thin:
                key = (layer, int(math.floor((p[0] + BOARD / 2) / region)), int(math.floor((p[1] + BOARD / 2) / region)))
                where[key] = where.get(key, 0.0) + L / n
    return {"by_width": dict(sorted(by_width.items())), "length": total, "narrow": narrow / total,
            "tight_space": tight / total, "narrow_mm": narrow, "tight_mm": tight, "where": where, "region": region}


def report(s):
    print("track length %.0f mm; by width: %s" % (s["length"], ", ".join("%.3f: %.0f mm" % kv for kv in s["by_width"].items())))
    print("narrower than 0.127 mm: %.0f mm (%.1f %%); space under 0.127 mm: %.0f mm (%.1f %%)" % (
        s["narrow_mm"], 100 * s["narrow"], s["tight_mm"], 100 * s["tight_space"]))
    n = int(round(BOARD / s["region"]))
    for layer in LAYERS.values():
        print("  %s: mm of track at the 0.1 mm minimum (width or space) per %.0f mm square, x left to right, y top to bottom" % (layer, s["region"]))
        for gy in range(n):
            print("    " + " ".join("%5.0f" % s["where"].get((layer, gx, gy), 0.0) for gx in range(n)))


def main(argv):
    if argv[0] == "--stats":
        report(stats(json.load(open(argv[1]))))
        return
    pre, routed = json.load(open(argv[0])), json.load(open(argv[1]))
    before = sum(math.dist(t["start"], t["end"]) for t in routed["tracks"] if t["width"] < SPACE - 1e-6)
    count = widen(pre, routed)
    after = sum(math.dist(t["start"], t["end"]) for t in routed["tracks"] if t["width"] < SPACE - 1e-6)
    json.dump(routed, open(argv[2], "w"), indent=0)
    print("widen: %s segments; track under 0.127 mm: %.0f mm -> %.0f mm" % (
        ", ".join("%d to %.3f" % (n, w) for w, n in sorted(count.items())), before, after))


if __name__ == "__main__":
    main(sys.argv[1:])

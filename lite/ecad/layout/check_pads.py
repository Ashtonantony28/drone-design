#!/usr/bin/env python3
"""Placement sanity check: pairs of copper pads of different nets closer than a gap (mm).

    python3 layout/check_pads.py layout/build/board.json [gap]
"""
import json
import math
import sys


def rects(d):
    out = []
    for fp in d["footprints"]:
        for p in fp["pads"]:
            if p["npth"] or not (fp["side"] == "B" or p["drill"] > 0):
                continue
            (x, y), (w, h) = p["at"], p["size"]
            if abs(math.sin(math.radians(p.get("angle", 0)))) > 0.7:
                w, h = h, w
            out.append(("%s.%s" % (fp["ref"], p["number"]), p["net"], x, y, w, h, p.get("shape") == "circle"))
    return out


def gap(a, b):
    gx = abs(a[2] - b[2]) - (a[4] + b[4]) / 2
    gy = abs(a[3] - b[3]) - (a[5] + b[5]) / 2
    if gx > 0 and gy > 0:
        return math.hypot(gx, gy)
    return max(gx, gy)


if __name__ == "__main__":
    d = json.load(open(sys.argv[1]))
    lim = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    r = rects(d)
    bad = []
    for i, a in enumerate(r):
        for b in r[i + 1:]:
            if a[1] == b[1] and a[1] or a[0].split(".")[0] == b[0].split(".")[0]:
                continue
            g = gap(a, b)
            if g < lim:
                bad.append((round(g, 2), a[0], a[1], b[0], b[1]))
    for x in sorted(bad):
        print(*x)
    hx = [(-19, -15), (19, -15), (19, 15), (-19, 15)]
    for a in r:
        for h in hx:
            if math.dist(h, a[2:4]) - max(a[4], a[5]) / 2 - 1.1 < 0.5:
                print("near hole", h, a[0], round(math.dist(h, a[2:4]) - max(a[4], a[5]) / 2 - 1.1, 2))
    # courtyard boxes of parts on the same side (B.CrtYd of the holes is not in the dump)
    fps = [f for f in d["footprints"] if f["courtyard"]]
    for i, a in enumerate(fps):
        for b in fps[i + 1:]:
            if a["side"] != b["side"]:
                continue
            ca, cb = a["courtyard"], b["courtyard"]
            ox = min(ca[2], cb[2]) - max(ca[0], cb[0])
            oy = min(ca[3], cb[3]) - max(ca[1], cb[1])
            if ox > 0 and oy > 0:
                print("courtyard", a["ref"], b["ref"], round(ox, 2), round(oy, 2))

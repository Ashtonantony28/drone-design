#!/usr/bin/env python3
"""List courtyard overlaps and parts inside a hole keep-out, from layout/dump.py output."""
import json
import math
import sys

d = json.load(open(sys.argv[1]))
fps = [f for f in d["footprints"] if f["courtyard"]]
bad = 0
for i, a in enumerate(fps):
    for b in fps[i + 1:]:
        if a["side"] != b["side"]:
            continue
        ca, cb = a["courtyard"], b["courtyard"]
        ox = min(ca[2], cb[2]) - max(ca[0], cb[0])
        oy = min(ca[3], cb[3]) - max(ca[1], cb[1])
        if ox > 0.001 and oy > 0.001 and not (a["ref"].startswith("H") or b["ref"].startswith("H")):
            bad += 1
            print("overlap %s %s (%s) %.2f x %.2f" % (a["ref"], b["ref"], a["side"], ox, oy))
for f in d["footprints"]:
    if f["ref"].startswith("H"):
        continue
    for p in f["pads"]:
        b = p["box"]
        for sx in (-15.25, 15.25):
            for sy in (-15.25, 15.25):
                dx = max(b[0] - sx, 0, sx - b[2])
                dy = max(b[1] - sy, 0, sy - b[3])
                if math.hypot(dx, dy) < 3.5:
                    bad += 1
                    print("pad %s.%s inside hole keep-out" % (f["ref"], p["number"]))
        if max(abs(b[0]), abs(b[2]), abs(b[1]), abs(b[3])) > 17.7 and not f["ref"] in ("J1",):
            bad += 1
            print("pad %s.%s within 0.3 mm of the edge" % (f["ref"], p["number"]))
print("%d problems" % bad)

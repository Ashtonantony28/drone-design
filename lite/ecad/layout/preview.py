#!/usr/bin/env python3
"""Quick look at a router result without KiCad: board.json + routes.json -> preview_*.png.

    python3 layout/preview.py layout/build/board.json layout/build/routes.json layout/build/preview
"""
import json
import sys

import plot

if __name__ == "__main__":
    d = json.load(open(sys.argv[1]))
    r = json.load(open(sys.argv[2]))
    d["tracks"] = [dict(t, len=0) for t in r["tracks"]]
    d["vias"] = [{"net": v["net"], "at": v["at"], "dia": v.get("dia", 2.0), "drill": v.get("drill", 1.0)} for v in r["vias"]]
    rats = plot.ratsnest(d)
    plot.draw(d, "F", sys.argv[3] + "_top.png", rats)
    plot.draw(d, "B", sys.argv[3] + "_bottom.png", rats)

#!/usr/bin/env python3
"""Place the Lite board: outline, mounting holes, parts, netclasses, mill DRC rules.

Runs under KiCad 10's Python (layout/run.sh place). Re-runnable: removes every
track, via and zone and re-places every footprint from geometry.PARTS.
"""
import json
import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geometry import BOARD_H, BOARD_W, CORNER_R, HOLES, ORIGIN, PARTS, POWER_NETS  # noqa: E402

MM = pcbnew.FromMM

# Extra mill rules that the .kicad_pro cannot express (MILLING.md). kicad-cli reads
# <board>.kicad_dru next to the board.
DRU = """(version 1)
# Mill rules for the NF-3D Lite flight controller (lite/ecad/MILLING.md).
# Signals stay on the bottom copper; the top copper carries only VBAT and GND
# (MILLING.md "Top side"), between top-solderable through-hole pads and wire vias.
(rule "top copper: VBAT and GND only"
    (layer F.Cu)
    (condition "A.Type == 'track' && A.NetName != 'VBAT' && A.NetName != 'GND'")
    (constraint disallow track))
# Battery and motor current: 1.0 mm minimum ...
(rule "power width"
    (condition "A.NetClass == 'Power'")
    (constraint track_width (min 1.0mm)))
# ... except the 0.5 mm stubs between the 0.95 mm pitch pins of the LDO (U4, SOT-23-5).
(rule "power width at U4"
    (condition "A.NetClass == 'Power' && A.intersectsCourtyard('U4')")
    (constraint track_width (min 0.5mm)))
# Layer changes only through 1.0 mm drilled wire vias.
(rule "wire vias"
    (condition "A.Type == 'via'")
    (constraint hole_size (min 1.0mm))
    (constraint via_diameter (min 2.0mm)))
"""

LIBRARY_CHECKS = ("lib_footprint_issues", "lib_footprint_mismatch")
SILK_REFS = ("J", "SW", "U", "M", "D5", "C1", "JP", "H")


def pt(x, y):
    return pcbnew.VECTOR2I(MM(ORIGIN[0] + x), MM(ORIGIN[1] + y))


def path_of(fp):
    return next((f.GetText() for f in fp.GetFields() if f.GetName() == "Path"), fp.GetReference())


def put(fp, x, y, rot, side):
    if fp.IsFlipped() != (side == "B"):
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    fp.SetPosition(pt(x, y))
    fp.SetOrientationDegrees(rot)


def set_outline(board):
    hx, hy, r = BOARD_W / 2, BOARD_H / 2, CORNER_R
    k = r * (1 - math.sqrt(0.5))

    def line(a, b):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pt(*a))
        s.SetEnd(pt(*b))
        return s

    def arc(a, m, b):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_ARC)
        s.SetArcGeometry(pt(*a), pt(*m), pt(*b))
        return s

    shapes = [
        line((-hx + r, -hy), (hx - r, -hy)), arc((hx - r, -hy), (hx - k, -hy + k), (hx, -hy + r)),
        line((hx, -hy + r), (hx, hy - r)), arc((hx, hy - r), (hx - k, hy - k), (hx - r, hy)),
        line((hx - r, hy), (-hx + r, hy)), arc((-hx + r, hy), (-hx + k, hy - k), (-hx, hy - r)),
        line((-hx, hy - r), (-hx, -hy + r)), arc((-hx, -hy + r), (-hx + k, -hy + k), (-hx + r, -hy)),
    ]
    for s in shapes:
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(MM(0.1))
        board.Add(s)


def add_text(board, text, x, y, layer, size=1.0):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetLayer(layer)
    t.SetPosition(pt(x, y))
    t.SetTextSize(pcbnew.VECTOR2I(MM(size), MM(size)))
    t.SetTextThickness(MM(size * 0.15))
    if layer in (pcbnew.B_SilkS, pcbnew.B_Cu):
        t.SetMirrored(True)
    board.Add(t)


def tidy_text(footprints):
    for fp in footprints:
        ref = fp.Reference()
        keep = fp.GetReference().startswith(SILK_REFS)
        fab = pcbnew.B_Fab if fp.IsFlipped() else pcbnew.F_Fab
        silk = pcbnew.B_SilkS if fp.IsFlipped() else pcbnew.F_SilkS
        ref.SetLayer(silk if keep else fab)
        ref.SetTextSize(pcbnew.VECTOR2I(MM(0.8), MM(0.8)))
        ref.SetTextThickness(MM(0.12))
        fp.Value().SetLayer(fab)
        fp.Value().SetVisible(False)
        if not keep:
            ref.SetPosition(fp.GetPosition())


def assign_netclasses(pro_path):
    with open(pro_path) as f:
        pro = json.load(f)
    patterns = pro["net_settings"].setdefault("netclass_patterns", [])
    patterns[:] = [p for p in patterns if p.get("netclass") != "Power"]
    patterns.extend({"netclass": "Power", "pattern": n} for n in POWER_NETS)
    # Footprints come from lite/ecad/lib through the Zener package cache, not from a library table.
    severities = pro["board"]["design_settings"].setdefault("rule_severities", {})
    for check in LIBRARY_CHECKS:
        severities[check] = "ignore"
    with open(pro_path, "w") as f:
        json.dump(pro, f, indent=2)
        f.write("\n")


def main(board_path):
    base = os.path.splitext(board_path)[0]
    assign_netclasses(base + ".kicad_pro")
    with open(base + ".kicad_dru", "w") as f:
        f.write(DRU)
    board = pcbnew.LoadBoard(board_path)
    footprints = list(board.GetFootprints())
    stale = list(board.GetTracks()) + list(board.Zones())
    stale += [d for d in board.GetDrawings() if d.GetLayer() in (pcbnew.Edge_Cuts, pcbnew.F_SilkS, pcbnew.B_SilkS, pcbnew.B_Cu)]
    for item in stale:
        board.Remove(item)
    set_outline(board)
    fps = {path_of(fp): fp for fp in footprints}
    for i, (x, y) in enumerate(HOLES):
        put(fps.pop("H%d" % (i + 1)), x, y, 0, "F")
    for path, (x, y, rot, side) in PARTS.items():
        put(fps.pop(path), x, y, rot, side)
    assert not fps, "unplaced footprints: %s" % sorted(fps)
    for fp in footprints:
        fp.SetLocked(True)
    tidy_text(footprints)
    add_text(board, "NF-3D Lite FC", 4.4, 5.6, pcbnew.F_SilkS, 1.0)
    add_text(board, "NOSE +X >", 4.4, -18.0, pcbnew.F_SilkS, 0.8)
    board.Save(board_path)
    print("placed %d footprints" % len(footprints))


if __name__ == "__main__":
    main(sys.argv[1])

#!/usr/bin/env python3
"""Put the router's copper on the board and pour bottom ground (KiCad Python).

    python3 layout/route_apply.py <board.kicad_pcb> <routes.json>

Expects a freshly placed board (run.sh apply runs place.py first).
The B.Cu GND pour (0.5 mm clearance, 0.5 mm minimum width, 0.6 mm thermal spokes on
through-hole pads, SMD pads joined solid)
leaves less copper to mill away and lowers the motor-current return resistance.
"""
import json
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geometry import BOARD_H, BOARD_W, EDGE_CLEARANCE, ORIGIN  # noqa: E402

MM = pcbnew.FromMM


def pt(x, y):
    return pcbnew.VECTOR2I(MM(ORIGIN[0] + x), MM(ORIGIN[1] + y))


def main(board_path, routes_path):
    board = pcbnew.LoadBoard(board_path)
    # KiCad 10.0.8's SWIG proxies break after a Remove(), so this script only adds:
    # run.sh re-runs place.py (which clears tracks and zones) first.
    assert not list(board.GetTracks()) and not list(board.Zones()), "board has copper: run place first"
    codes = {str(name): net.GetNetCode() for name, net in board.GetNetsByName().items()}
    data = json.load(open(routes_path))
    layers = {"B.Cu": pcbnew.B_Cu, "F.Cu": pcbnew.F_Cu}
    for t in data["tracks"]:
        tr = pcbnew.PCB_TRACK(board)
        tr.SetStart(pt(*t["start"]))
        tr.SetEnd(pt(*t["end"]))
        tr.SetWidth(MM(t["width"]))
        tr.SetLayer(layers[t["layer"]])
        tr.SetNetCode(codes[t["net"]])
        board.Add(tr)
    for v in data["vias"]:
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(pt(*v["at"]))
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        via.SetWidth(pcbnew.F_Cu, MM(v["dia"]))
        via.SetDrill(MM(v["drill"]))
        via.SetNetCode(codes[v["net"]])
        board.Add(via)
    z = pcbnew.ZONE(board)
    z.SetLayer(pcbnew.B_Cu)
    z.SetNetCode(codes["GND"])
    z.SetZoneName("B_GND")
    z.SetLocalClearance(MM(0.5))
    z.SetMinThickness(MM(0.5))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)  # SMD pads solid, THT pads on spokes
    z.SetThermalReliefGap(MM(0.5))
    z.SetThermalReliefSpokeWidth(MM(0.6))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    e = EDGE_CLEARANCE
    hx, hy = BOARD_W / 2, BOARD_H / 2
    poly = z.Outline()
    poly.NewOutline()
    for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        poly.Append(MM(ORIGIN[0] + x), MM(ORIGIN[1] + y))
    board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(board_path)
    print("applied %d tracks, %d vias, GND pour" % (len(data["tracks"]), len(data["vias"])))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])

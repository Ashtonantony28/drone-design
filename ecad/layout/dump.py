#!/usr/bin/env python3
"""Dump board geometry as JSON (mm, relative to the board centre). Runs under KiCad's Python.

    python3 layout/dump.py <board.kicad_pcb> <out.json>

Used by layout/plot.py (review pictures) and check_layout.py (rule checks), so
that neither needs pcbnew.
"""
import json
import sys

import pcbnew

mm = pcbnew.ToMM


def main(board_path, out_path):
    b = pcbnew.LoadBoard(board_path)
    bb = b.GetBoardEdgesBoundingBox()
    ox, oy = mm(bb.GetCenter().x), mm(bb.GetCenter().y)

    def xy(v):
        return [round(mm(v.x) - ox, 4), round(mm(v.y) - oy, 4)]

    def box(r):
        return [round(mm(r.GetLeft()) - ox, 4), round(mm(r.GetTop()) - oy, 4), round(mm(r.GetRight()) - ox, 4), round(mm(r.GetBottom()) - oy, 4)]

    out = {"edge": box(bb), "footprints": [], "tracks": [], "vias": [], "zones": [], "edges": []}
    for d in b.GetDrawings():
        if d.GetLayer() == pcbnew.Edge_Cuts:
            out["edges"].append({"shape": d.ShowShape(), "start": xy(d.GetStart()), "end": xy(d.GetEnd()),
                                 "center": xy(d.GetCenter()) if d.GetShape() == pcbnew.SHAPE_T_ARC else None})
    for fp in b.GetFootprints():
        side = "B" if fp.IsFlipped() else "F"
        cy = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
        pads = []
        for p in fp.Pads():
            pads.append({
                "number": p.GetNumber(), "net": p.GetNetname(), "at": xy(p.GetPosition()), "box": box(p.GetBoundingBox()),
                "drill": mm(p.GetDrillSize().x), "plated": p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH,
                "npth": p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH,
                "layers": [n for n, l in (("F", pcbnew.F_Cu), ("B", pcbnew.B_Cu)) if p.IsOnLayer(l)],
            })
        out["footprints"].append({
            "ref": fp.GetReference(), "path": next((f.GetText() for f in fp.GetFields() if f.GetName() == "Path"), fp.GetReference()),
            "side": side, "at": xy(fp.GetPosition()), "rot": fp.GetOrientationDegrees(), "dnp": fp.IsDNP(),
            "footprint": fp.GetFPID().GetUniStringLibItemName(),
            "courtyard": box(cy.BBox()) if cy.OutlineCount() else None, "pads": pads,
        })
    for t in b.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            out["vias"].append({"net": t.GetNetname(), "at": xy(t.GetPosition()), "dia": mm(t.GetWidth(pcbnew.F_Cu)), "drill": mm(t.GetDrillValue())})
        else:
            out["tracks"].append({"net": t.GetNetname(), "layer": b.GetLayerName(t.GetLayer()), "start": xy(t.GetStart()), "end": xy(t.GetEnd()),
                                  "width": mm(t.GetWidth()), "len": mm(t.GetLength())})
    for z in b.Zones():
        o = z.Outline()
        pts = [xy(o.CVertex(i)) for i in range(o.TotalVertices())]
        layers = [b.GetLayerName(l) for l in z.GetLayerSet().Seq()]
        filled = {}
        if not z.GetIsRuleArea():
            for l in z.GetLayerSet().Seq():
                poly = z.GetFilledPolysList(l)
                filled[b.GetLayerName(l)] = {"area": round(poly.Area() / 1e12, 3), "outlines": poly.OutlineCount(),
                                             "holes": sum(poly.HoleCount(i) for i in range(poly.OutlineCount()))}
        out["zones"].append({"name": z.GetZoneName(), "net": z.GetNetname(), "layers": layers, "keepout": z.GetIsRuleArea(), "outline": pts, "filled": filled})
    with open(out_path, "w") as f:
        json.dump(out, f)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])

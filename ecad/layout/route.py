"""Routing steps for nf_fc that need KiCad's Python (run through layout/run.sh).

  route.py apply  <board> <preroute.json>   add the scripted copper from preroute.py (locked)
  route.py export <board> <out.dsn>         Specctra DSN for Freerouting
  route.py import <board> <in.ses>          read the Freerouting session back
  route.py finish <board>                   outer-layer pours, refill, save

Coordinates in the JSON are mm from the board centre (see place.py).
"""
import json
import sys

import pcbnew

from place import BOARD_SIZE, EDGE_INSET, ORIGIN, add_zone, pt, rect

MM = pcbnew.FromMM
LAYER = {"F": pcbnew.F_Cu, "B": pcbnew.B_Cu, "I": pcbnew.In2_Cu, "F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu,
         "In1.Cu": pcbnew.In1_Cu, "In2.Cu": pcbnew.In2_Cu}


def add_track(board, t, locked):
    tr = pcbnew.PCB_TRACK(board)
    tr.SetStart(pt(*t["start"]))
    tr.SetEnd(pt(*t["end"]))
    tr.SetWidth(MM(t["width"]))
    tr.SetLayer(LAYER[t["layer"]])
    tr.SetNetCode(board.GetNetcodeFromNetname(t["net"]))
    tr.SetLocked(locked)
    board.Add(tr)


def add_via(board, v, locked):
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(pt(*v["at"]))
    via.SetViaType(pcbnew.VIATYPE_THROUGH)
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    via.SetWidth(pcbnew.F_Cu, MM(v["dia"]))
    via.SetDrill(MM(v["drill"]))
    via.SetNetCode(board.GetNetcodeFromNetname(v["net"]))
    via.SetLocked(locked)
    board.Add(via)


def apply(board_path, json_path, locked="locked"):
    """Add tracks and vias from a JSON file: the scripted pre-route (locked,
    onto the bare placement) or the maze router's output (unlocked, on top)."""
    locked = locked == "locked"
    board = pcbnew.LoadBoard(board_path)
    if locked:
        assert not list(board.GetTracks()), "board already has tracks: run place first"
    data = json.load(open(json_path))
    for t in data["tracks"]:
        add_track(board, t, locked)
    for v in data["vias"]:
        add_via(board, v, locked)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(board_path)
    print("applied %d tracks, %d vias" % (len(data["tracks"]), len(data["vias"])))


def export(board_path, dsn_path):
    board = pcbnew.LoadBoard(board_path)
    if not pcbnew.ExportSpecctraDSN(board, dsn_path):
        sys.exit("DSN export failed")
    print("wrote", dsn_path)


def parse_sexpr(text):
    """Minimal s-expression reader for Specctra session files."""
    out, stack, i, n = [], [], 0, len(text)
    cur = out
    while i < n:
        c = text[i]
        if c == "(":
            new = []
            cur.append(new)
            stack.append(cur)
            cur = new
            i += 1
        elif c == ")":
            cur = stack.pop()
            i += 1
        elif c == '"':
            j = text.index('"', i + 1)
            cur.append(text[i + 1:j])
            i = j + 1
        elif c.isspace():
            i += 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in "()":
                j += 1
            cur.append(text[i:j])
            i = j
    return out[0]


def read_session(ses_path):
    """Wires and vias of a Specctra session as preroute.json-style dicts."""
    tree = parse_sexpr(open(ses_path).read())
    routes = next(x for x in tree if isinstance(x, list) and x[0] == "routes")
    res = next(x for x in routes if isinstance(x, list) and x[0] == "resolution")
    unit = {"um": 1e-3, "mm": 1.0, "mil": 0.0254}[res[1]] / float(res[2])
    ox, oy = ORIGIN

    def xy(x, y):
        return [round(float(x) * unit - ox, 4), round(-float(y) * unit - oy, 4)]

    tracks, vias = [], []
    net_out = next(x for x in routes if isinstance(x, list) and x[0] == "network_out")
    for net in net_out[1:]:
        name = net[1]
        for item in net[2:]:
            if item[0] == "wire":
                path = item[1]
                layer, width, pts = path[1], float(path[2]) * unit, path[3:]
                for k in range(0, len(pts) - 2, 2):
                    tracks.append({"net": name, "layer": layer, "width": round(width, 4),
                                   "start": xy(pts[k], pts[k + 1]), "end": xy(pts[k + 2], pts[k + 3])})
            elif item[0] == "via":
                dia, drill = item[1].split("_")[1].split(":")  # "Via[0-3]_450:300_um"
                vias.append({"net": name, "at": xy(item[2], item[3]), "dia": float(dia) / 1000, "drill": float(drill) / 1000})
    return tracks, vias


def import_ses(board_path, ses_path):
    """Add the autorouter's wires and vias to the pre-routed board.

    pcbnew.ImportSpecctraSES crashes when called from a script in KiCad 10.0.8,
    so the session file is read here.  The scripted copper went out in the DSN
    as fixed wires and is not repeated in the session.
    """
    board = pcbnew.LoadBoard(board_path)
    have = set()
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            have.add(("via", t.GetPosition().x, t.GetPosition().y))
    tracks, vias = read_session(ses_path)
    n = 0
    for t in tracks:
        if t["start"] != t["end"]:
            add_track(board, t, False)
            n += 1
    for v in vias:
        p = pt(*v["at"])
        if ("via", p.x, p.y) not in have:
            add_via(board, v, False)
            n += 1
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(board_path)
    print("imported %s: %d autorouted tracks and vias" % (ses_path, n))


# Power pours on L4 (B.Cu), added after routing on top of the tracks of their net: the fill
# keeps its clearance to everything else, so a pour can only add copper where there is room.
# name -> (net, outline).  None of them comes near the IMU island (check_layout.py tests it),
# and L2 stays the unbroken ground plane.
POURS = {
    # battery input: J2 pin 1 -> TVS D3 -> the buck's input capacitors on the J2 side
    "L4_VBAT_IN": ("VBAT", [(8.85, 3.6), (17.0, 3.6), (17.0, 5.8), (12.9, 5.8), (12.9, 10.2), (9.6, 10.2),
                            (9.6, 13.4), (8.8, 13.4), (8.8, 8.7), (10.0, 8.7), (10.0, 5.8), (8.85, 5.8)]),
    # the input capacitors on the other side of the buck (joined to the first group on L3)
    "L4_VBAT_HF": ("VBAT", [(3.2, 3.6), (8.05, 3.6), (8.05, 4.7), (6.9, 4.9), (5.2, 5.75), (3.2, 5.75)]),
    # buck output: inductor -> the three output capacitors
    "L4_5V_OUT": ("5V", [(9.5, -2.2), (10.7, -2.2), (10.7, -0.15), (17.3, -0.15), (17.3, 0.95), (10.7, 0.95), (10.7, 1.6), (9.5, 1.6)]),
}
POUR_CLEARANCE = 0.2
PLANE_NETS = ("GND", "3V3_D", "3V3_IMU")


def prune_vias(board, tracks, footprints):
    """Take out vias that ended up with copper on one layer only (an escape via scripted by
    preroute.py that the maze router then did not use): they would only be a drill hit."""
    segs = {}
    for t in tracks:
        if t.Type() != pcbnew.PCB_VIA_T:
            a, b = t.GetStart(), t.GetEnd()
            segs.setdefault(t.GetNetCode(), []).append((a.x, a.y, b.x, b.y, t.GetLayer()))
    pads = {}
    for fp in footprints:
        for p in fp.Pads():
            pads.setdefault(p.GetNetCode(), []).append(p)

    def touches(c, r, seg):
        # a track may pass over the via without ending on it (a T junction)
        ax, ay, bx, by, _ = seg
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((c.x - ax) * vx + (c.y - ay) * vy) / L2))
        return (ax + t * vx - c.x) ** 2 + (ay + t * vy - c.y) ** 2 <= r * r

    gone = []
    for v in tracks:
        if v.Type() != pcbnew.PCB_VIA_T or v.GetNetname() in PLANE_NETS:
            continue
        c, r = v.GetPosition(), v.GetWidth(pcbnew.F_Cu) / 2
        layers = {seg[4] for seg in segs.get(v.GetNetCode(), []) if touches(c, r, seg)}
        for p in pads.get(v.GetNetCode(), []):
            if p.GetBoundingBox().Contains(c):
                layers.update(l for l in (pcbnew.F_Cu, pcbnew.B_Cu) if p.IsOnLayer(l))
        if len(layers) < 2:
            gone.append(v)
    names = ["%s @(%.2f, %.2f)" % (v.GetNetname(), pcbnew.ToMM(v.GetPosition().x) - ORIGIN[0], pcbnew.ToMM(v.GetPosition().y) - ORIGIN[1]) for v in gone]
    return gone, names


def finish(board_path):
    import silk
    board = pcbnew.LoadBoard(board_path)
    # take every container before the first Remove() (see place.py)
    tracks, footprints, zones = list(board.GetTracks()), list(board.GetFootprints()), list(board.Zones())
    names = {z.GetZoneName() for z in zones}
    e = BOARD_SIZE / 2 - EDGE_INSET
    if "L1_GND" not in names:
        add_zone(board, "GND", pcbnew.F_Cu, rect(-e, -e, e, e), name="L1_GND", solid=True, clearance=0.2)
    if "L4_GND" not in names:
        add_zone(board, "GND", pcbnew.B_Cu, rect(-e, -e, e, e), name="L4_GND", solid=True, clearance=0.2)
    for name, (net, outline) in POURS.items():
        if name not in names:
            add_zone(board, net, pcbnew.B_Cu, outline, priority=1, name=name, solid=True, clearance=POUR_CLEARANCE)
    gone, pruned = prune_vias(board, tracks, footprints)
    silk.tidy(board)
    for v in gone:
        board.Remove(v)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(board_path)
    print("finished: zones", sorted(names | {"L1_GND", "L4_GND"} | set(POURS)))
    print("finished: %d dangling vias removed%s" % (len(pruned), (": " + ", ".join(pruned)) if pruned else ""))


if __name__ == "__main__":
    cmd, args = sys.argv[1], sys.argv[2:]
    {"apply": apply, "export": export, "import": import_ses, "finish": finish}.get(cmd, lambda *a: sys.exit(__doc__))(*args)

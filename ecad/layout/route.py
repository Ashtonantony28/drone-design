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


def finish(board_path):
    board = pcbnew.LoadBoard(board_path)
    names = {z.GetZoneName() for z in board.Zones()}
    e = BOARD_SIZE / 2 - EDGE_INSET
    if "L1_GND" not in names:
        add_zone(board, "GND", pcbnew.F_Cu, rect(-e, -e, e, e), name="L1_GND", solid=True, clearance=0.2)
    if "L4_GND" not in names:
        add_zone(board, "GND", pcbnew.B_Cu, rect(-e, -e, e, e), name="L4_GND", solid=True, clearance=0.2)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(board_path)
    print("finished: zones", sorted(z.GetZoneName() for z in board.Zones()))


if __name__ == "__main__":
    cmd, args = sys.argv[1], sys.argv[2:]
    {"apply": apply, "export": export, "import": import_ses, "finish": finish}.get(cmd, lambda *a: sys.exit(__doc__))(*args)

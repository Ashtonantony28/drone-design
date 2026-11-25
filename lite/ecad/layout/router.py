#!/usr/bin/env python3
"""Single-sided grid router for the Lite board (host Python: numpy + scipy).

    python3 layout/router.py <board.json> <routes.json>

Routes every net on B.Cu with the mill rules of geometry.py (0.5 mm signal,
1.0 mm power, 0.4 mm clearance; the router keeps 0.45 mm). A top-side wire jumper
(an F.Cu track between two 1.0 mm drilled wire vias or through-hole pads) is only
used when a net cannot be finished on the bottom; jumpers cost JUMPER_COST per
cell plus VIA_COST per wire via, so the router takes them only as a last resort, and
prefers a top track between top-solderable through-hole pads to a wire via.

Method: 0.1 mm grid. For each net, a Euclidean distance transform of all
other-net copper (and the board edge and holes) gives the cells where a track
of the net's width keeps its clearance; A* (8 directions, a small penalty per
bend) grows a tree from the net's first pad to each next-nearest pad. Nets that
fail are moved to the front and the board is routed again (ROUNDS times).
Pre-routes (PREROUTE below) are fixed copper laid before anything else.
"""
import heapq
import json
import math
import os
import sys
import time

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geometry import BOARD_H, BOARD_W, CORNER_R, POWER_NETS  # noqa: E402

G = 0.1  # grid pitch, mm
CLR = 0.45  # copper clearance kept by the router (rule: 0.4)
EDGE_CLR = 0.55  # rule 0.5
HOLE_CLR = 0.55  # rule 0.5
MARGIN = 0.075  # cell discretisation (0.71 G)
VIA_DIA, VIA_DRILL = 2.2, 1.0
JUMPER_COST = 2.5
# Through-hole pads whose top side can be soldered (wire pads, leaded parts on stand-off, the
# TP4056 wire legs).  Header, switch and JST pins sit under a plastic body: no layer change there.
TOP_SOLDERABLE = ("M1", "M2", "M3", "M4", "C1", "D5", "U3")
VIA_COST = 400.0
BEND = 0.4
ROUNDS = int(os.environ.get("ROUTER_ROUNDS", 12))
VERBOSE = bool(os.environ.get("ROUTER_VERBOSE"))

WIDTH = {n: 1.0 for n in POWER_NETS}
WIDTH["3V3"] = 0.8
FALLBACK = {"3V3": 0.5}
DEFAULT_W = 0.5

PAD = 1.0  # grid margin outside the board: the edge distance transform needs outside cells
NX = int(round((BOARD_W + 2 * PAD) / G)) + 1
NY = int(round((BOARD_H + 2 * PAD) / G)) + 1
X0, Y0 = -BOARD_W / 2 - PAD, -BOARD_H / 2 - PAD
B, F = 0, 1


def ij(x, y):
    return int(round((y - Y0) / G)), int(round((x - X0) / G))


def xy(i, j):
    return round(X0 + j * G, 4), round(Y0 + i * G, 4)


YY, XX = np.mgrid[0:NY, 0:NX]
XS = X0 + XX * G
YS = Y0 + YY * G


def inside_board():
    hx, hy, r = BOARD_W / 2, BOARD_H / 2, CORNER_R
    dx = np.maximum(np.abs(XS) - (hx - r), 0)
    dy = np.maximum(np.abs(YS) - (hy - r), 0)
    return (np.abs(XS) <= hx) & (np.abs(YS) <= hy) & (dx * dx + dy * dy <= r * r)


def pad_mask(p, grow=0.0):
    (x, y), (w, h) = p["at"], p["size"]
    if abs(math.sin(math.radians(p.get("angle", 0)))) > 0.7:
        w, h = h, w
    w, h = w + 2 * grow, h + 2 * grow
    dx, dy = XS - x, YS - y
    if p["shape"] == "circle":
        return dx * dx + dy * dy <= (w / 2) ** 2
    if p["shape"] == "oval":
        r = min(w, h) / 2
        ex, ey = max(w / 2 - r, 0), max(h / 2 - r, 0)
        qx, qy = np.maximum(np.abs(dx) - ex, 0), np.maximum(np.abs(dy) - ey, 0)
        return qx * qx + qy * qy <= r * r
    return (np.abs(dx) <= w / 2) & (np.abs(dy) <= h / 2)


def seg_mask(a, b, half):
    (ax, ay), (bx, by) = a, b
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = np.zeros_like(XS) if L2 == 0 else np.clip(((XS - ax) * vx + (YS - ay) * vy) / L2, 0, 1)
    px, py = ax + t * vx - XS, ay + t * vy - YS
    return px * px + py * py <= half * half


class Board:
    def __init__(self, data):
        self.inside = inside_board()
        self.edge_dist = ndimage.distance_transform_edt(self.inside) * G  # distance to outside, mm
        self.holes = np.zeros((NY, NX), bool)
        self.label = [np.zeros((NY, NX), np.int32), np.zeros((NY, NX), np.int32)]  # copper owner per layer
        self.nets = {"": 0}
        self.pads = {}  # net -> list of pads
        self.tracks, self.vias = [], []
        self.transfer = np.zeros((NY, NX), bool)  # top-solderable through-hole pad cells
        self.drilled = np.zeros((NY, NX), bool)  # every through-hole pad, of any net
        for fp in data["footprints"]:
            for p in fp["pads"]:
                p["ref"] = fp["ref"]
                if p["npth"]:
                    self.holes |= pad_mask({"at": p["at"], "size": [p["drill"]] * 2, "shape": "circle"})
                    continue
                nid = self.net_id(p["net"]) if p["net"] else -1  # -1: copper of no net (blocks everyone)
                m = pad_mask(p)
                for layer in (B, F):
                    if ("B" if layer == B else "F") in p["layers"]:
                        self.label[layer][m] = nid
                if p["drill"] > 0:
                    self.drilled |= m
                if p["drill"] > 0 and fp["ref"] in TOP_SOLDERABLE and nid > 0:
                    self.transfer[m] = True
                if p["net"] and not p["net"].split(".")[-1].startswith("NC_"):
                    self.pads.setdefault(p["net"], []).append(p)
        self.hole_dist = ndimage.distance_transform_edt(~self.holes) * G
        self.pad_dist = ndimage.distance_transform_edt(~self.drilled) * G  # a wire via never sits on a pad

    def net_id(self, name):
        if name not in self.nets:
            self.nets[name] = len(self.nets)
        return self.nets[name]

    def free_masks(self, net, width):
        """Per layer: cells where a track of `width` (or a via, key 'via') clears everything else."""
        nid = self.nets[net]
        out = {}
        for layer in (B, F):
            lab = self.label[layer]
            other = (lab != 0) & (lab != nid)
            d = ndimage.distance_transform_edt(~other) * G
            base = (self.edge_dist >= 0) & self.inside
            out[layer] = {}
            for key, half in (("w", width / 2), ("via", VIA_DIA / 2)):
                out[layer][key] = (base & (d >= half + CLR + MARGIN) & (self.edge_dist >= half + EDGE_CLR + MARGIN)
                                   & (self.hole_dist >= half + HOLE_CLR + MARGIN))
            out[layer]["via"] &= self.pad_dist >= VIA_DIA / 2 + CLR + MARGIN
        return out

    def add_track(self, net, layer, a, b, width):
        nid = self.nets[net]
        self.label[layer][seg_mask(a, b, width / 2)] = nid
        self.tracks.append({"net": net, "layer": "B.Cu" if layer == B else "F.Cu", "width": width,
                            "start": list(a), "end": list(b)})

    def add_via(self, net, at):
        nid = self.nets[net]
        m = pad_mask({"at": at, "size": [VIA_DIA, VIA_DIA], "shape": "circle"})
        self.label[B][m] = nid
        self.label[F][m] = nid
        self.vias.append({"net": net, "at": list(at), "dia": VIA_DIA, "drill": VIA_DRILL})


NN = NX * NY
SQ2 = math.sqrt(2)
OFFS = [(-NX, 1.0), (NX, 1.0), (-1, 1.0), (1, 1.0), (-NX - 1, SQ2), (-NX + 1, SQ2), (NX - 1, SQ2), (NX + 1, SQ2)]


def astar(fw, fv, ownpad, sources, target, tc, allow_f):
    """Grid A* on flat cell indices (layer * NN + i * NX + j).

    fw[layer]: bytes, 1 where a track centre may go; fv: bytes, 1 where a wire via fits;
    ownpad: bytes, 1 on this net's through-hole pads (free layer change); sources /
    target: sets of flat indices. The board edge cells are never free, so the flat
    neighbour offsets cannot wrap around a row.
    """
    ti, tj = tc
    heap = []
    best, parent, dirof = {}, {}, {}
    push, pop = heapq.heappush, heapq.heappop

    def h(c):
        r = c % NN
        di, dj = abs(r // NX - ti), abs(r % NX - tj)
        return di + dj + (SQ2 - 2) * min(di, dj)

    for c in sources:
        best[c] = 0.0
        parent[c] = -1
        dirof[c] = -1
        push(heap, (h(c), 0.0, c))
    while heap:
        f, g, c = pop(heap)
        if g > best[c]:
            continue
        if c in target:
            path = []
            while c != -1:
                path.append(c)
                c = parent[c]
            return path[::-1]
        layer = c // NN
        r = c - layer * NN
        free = fw[layer]
        d0 = dirof[c]
        sc = 1.0 if layer == B else JUMPER_COST
        for k in range(8):
            off, cost = OFFS[k]
            nr = r + off
            if nr < 0 or nr >= NN:
                continue
            n = c + off
            if not free[nr] and n not in target:
                continue
            ng = g + cost * sc + (BEND if (d0 >= 0 and d0 != k) else 0.0)
            if ng < best.get(n, 1e18):
                best[n] = ng
                parent[n] = c
                dirof[n] = k
                push(heap, (ng + h(n), ng, n))
        if allow_f:
            n = (NN + r) if layer == B else r
            if ownpad[r] or fv[r]:
                ng = g + (2.0 if ownpad[r] else VIA_COST)
                if ng < best.get(n, 1e18):
                    best[n] = ng
                    parent[n] = c
                    dirof[n] = -1
                    push(heap, (ng + h(n), ng, n))
    return None


def simplify(path):
    """Flat-index path -> list of (layer, [points]) runs with collinear points merged."""
    cells = [(c // NN, (c % NN) // NX, (c % NN) % NX) for c in path]
    runs, cur = [], [cells[0]]
    for n in cells[1:]:
        if n[0] != cur[-1][0]:
            runs.append(cur)
            cur = [n]
        else:
            cur.append(n)
    runs.append(cur)
    out = []
    for run in runs:
        pts = [run[0]]
        for k in range(1, len(run) - 1):
            a, b, c = pts[-1], run[k], run[k + 1]
            if (b[1] - a[1]) * (c[2] - b[2]) != (b[2] - a[2]) * (c[1] - b[1]):
                pts.append(b)
        if len(run) > 1:
            pts.append(run[-1])
        out.append((run[0][0], [xy(p[1], p[2]) for p in pts]))
    return out


def flat(layer, mask):
    return set((np.flatnonzero(mask) + layer * NN).tolist())


def pad_target(p, free, allow_f):
    """Cells of the pad where a track of this width may end, plus the pad centre."""
    m = pad_mask(p, grow=-0.05)
    ci, cj = ij(*p["at"])
    cells = set()
    for layer in (B, F):
        if ("B" if layer == B else "F") not in p["layers"] or (layer == F and not allow_f):
            continue
        if layer == F and p.get("ref") not in TOP_SOLDERABLE:
            continue
        cells |= flat(layer, m & free[layer]["w"])
        cells.add(layer * NN + ci * NX + cj)
    return cells, (ci, cj)


def groups_of(board, net, free, allow_f):
    """The net's pads, joined into groups by copper already laid (pre-routes).

    Each group: {"cells": flat cells where a track may start/end, "pts": points, "name": label}.
    """
    pads = list(board.pads[net])
    tracks = []
    for t in board.tracks:
        if t["net"] != net:
            continue
        layer = B if t["layer"] == "B.Cu" else F
        m = seg_mask(t["start"], t["end"], t["width"] / 2)
        lname = "B" if layer == B else "F"
        touch = [k for k, p in enumerate(pads) if lname in p["layers"] and (pad_mask(p, grow=0.05) & m).any()]
        tracks.append((layer, m, touch, t))
    # union-find over pads (0..n-1) and tracks (n..): a track joins the pads it touches
    # and the tracks of its layer that it overlaps
    n = len(pads)
    parent = list(range(n + len(tracks)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for x, (_, _, touch, _) in enumerate(tracks):
        for k in touch:
            parent[find(n + x)] = find(k)
    def bbox(t):
        h = t["width"] / 2
        return (min(t["start"][0], t["end"][0]) - h, min(t["start"][1], t["end"][1]) - h,
                max(t["start"][0], t["end"][0]) + h, max(t["start"][1], t["end"][1]) + h)

    boxes = [bbox(t[3]) for t in tracks]
    for x in range(len(tracks)):
        for y in range(x + 1, len(tracks)):
            a, b = boxes[x], boxes[y]
            if a[0] > b[2] or b[0] > a[2] or a[1] > b[3] or b[1] > a[3] or tracks[x][0] != tracks[y][0]:
                continue
            if (tracks[x][1] & tracks[y][1]).any():
                parent[find(n + x)] = find(n + y)
    groups = {}
    for k, p in enumerate(pads):
        g = groups.setdefault(find(k), {"cells": set(), "pts": [], "name": "%s.%s" % (p["ref"], p["number"])})
        cells, _ = pad_target(p, free, allow_f)
        g["cells"] |= cells
        g["pts"].append(tuple(p["at"]))
    for x, (layer, m, touch, t) in enumerate(tracks):
        if (layer == F and not allow_f) or find(n + x) not in groups:
            continue
        g = groups[find(n + x)]
        g["cells"] |= flat(layer, m & free[layer]["w"])
        g["pts"] += [tuple(t["start"]), tuple(t["end"])]
    return list(groups.values())


def route_net(board, net, width, allow_f, log):
    free = board.free_masks(net, width)
    groups = groups_of(board, net, free, allow_f)
    if len(groups) < 2:
        return True
    fw = [free[B]["w"].ravel().tobytes(), free[F]["w"].ravel().tobytes()]
    fv = (free[B]["via"] & free[F]["via"]).ravel().tobytes()
    nid = board.nets[net]
    cx = sum(q[0] for g in groups for q in g["pts"]) / sum(len(g["pts"]) for g in groups)
    cy = sum(q[1] for g in groups for q in g["pts"]) / sum(len(g["pts"]) for g in groups)
    # start from the group with room to leave it (most free cells), nearest the centroid on a tie
    groups.sort(key=lambda g: (-min(len(g["cells"]), 50), min(math.dist(q, (cx, cy)) for q in g["pts"])))
    first = groups.pop(0)
    tree = set(first["cells"])
    tree_pts = list(first["pts"])
    pending, progress = [], True
    while groups or (pending and progress):
        if not groups:  # retry the groups that failed, now that the tree is bigger
            groups, pending, progress = pending, [], False
        groups.sort(key=lambda g: min(math.dist(a, b) for a in g["pts"] for b in tree_pts[::max(1, len(tree_pts) // 200)]))
        g = groups.pop(0)
        target = g["cells"]
        if target & tree:
            tree |= target
            tree_pts += g["pts"]
            progress = True
            continue
        # aim the heuristic at the group point nearest the tree
        tc = ij(*min(g["pts"], key=lambda a: min(math.dist(a, b) for b in tree_pts[::max(1, len(tree_pts) // 200)])))
        ownpad = (board.transfer & (board.label[B] == nid) & (board.label[F] == nid)).ravel().tobytes()
        path = astar(fw, fv, ownpad, tree, target, tc, allow_f)
        if path is None:
            pending.append(g)
            continue
        progress = True
        runs = simplify(path)
        for k, (layer, pts) in enumerate(runs):
            for a, b in zip(pts, pts[1:]):
                board.add_track(net, layer, a, b, width)
                tree_pts.append(b)
            if k > 0 and runs[k - 1][0] != layer:
                p0 = runs[k - 1][1][-1]
                if not board.transfer[ij(*p0)]:
                    board.add_via(net, p0)
        for layer, pts in runs:
            for a, b in zip(pts, pts[1:]):
                tree |= flat(layer, seg_mask(a, b, width / 2) & free[layer]["w"])
        tree |= target
        tree_pts += g["pts"]
    for g in pending:
        log.append("  %s: no path to %s" % (net, g["name"]))
    return not pending


def preroute(data):
    """Fixed copper laid before routing: (net, [points], width, layer).

    U4 (ME6211, SOT-23-5): CE (pin 3) joins VIN (pin 1) under the body, and 0.5 mm
    stubs lead VIN, GND and VOUT out of the 0.95 mm pin pitch so that the 1.0 mm
    power tracks can land clear of the neighbouring pins (lite_fc.kicad_dru allows
    0.5 mm power tracks inside U4's courtyard).

    VBAT top-side tracks (MILLING.md, "Top side"): VBAT has to reach all four corners
    and every other net fans out from the ESP32 in the middle, so on one layer VBAT
    would have to cross them.  Its spine runs instead on the top copper, from motor
    pad to motor pad (no wire vias): M1+ -> M3+ -> along the -y edge -> M4+.  The
    bottom carries the rest of VBAT (M4+ -> C1 -> M2+, the flyback diodes, the LDO,
    the divider); the router joins what is left with top tracks between wire vias.
    """
    fps = {f["ref"]: f for f in data["footprints"]}
    pad = {p["number"]: p["at"] for p in fps["U4"]["pads"]}
    cx, cy = fps["U4"]["at"]
    p1, p2, p3, p5 = pad["1"], pad["2"], pad["3"], pad["5"]
    s = 1 if cx > p1[0] else -1  # towards the body from the VIN/VSS/CE row
    dy = 1 if p3[1] > p1[1] else -1  # from pin 1 towards pin 3
    out = [
        ("VBAT", [(p1[0] + s * 0.45, p1[1]), (cx, p1[1]), (cx, p3[1]), (p3[0] + s * 0.45, p3[1])], 0.5, B),
        ("VBAT", [tuple(p3), (p3[0], p3[1] + dy * 1.6)], 0.5, B),
        ("GND", [tuple(p2), (p2[0] - s * 1.6, p2[1])], 0.5, B),
        ("3V3", [tuple(p5), (p5[0] + s * 1.4, p5[1])], 0.5, B),
    ]

    def at(ref, num):
        return tuple(next(p["at"] for p in fps[ref]["pads"] if p["number"] == num))

    m1, m3, m4 = at("M1", "1"), at("M3", "1"), at("M4", "1")
    xl, xh, xr, ytop = -18.6, -16.6, 16.6, -17.7
    out += [
        ("VBAT", [m1, (xl, m1[1] - 1.5), (xl, m3[1] - 0.5), m3], 1.0, F),
        ("VBAT", [(xl, m3[1] - 0.5), (xh, m3[1] - 2.5), (xh, ytop), (xr, ytop), (xr, m4[1] - 1.5), m4], 1.0, F),
    ]
    return out


def order_nets(board):
    """Short local nets first, the long power trees last."""
    def span(net):
        ps = [p["at"] for p in board.pads[net]]
        return (max(p[0] for p in ps) - min(p[0] for p in ps)) + (max(p[1] for p in ps) - min(p[1] for p in ps))
    nets = [n for n in board.pads if len(board.pads[n]) > 1]
    return sorted(nets, key=lambda n: (n in ("VBAT", "GND"), n == "GND", span(n)))


def run(data, order, allow_f_for=()):
    board = Board(json.loads(json.dumps(data)))
    for net, pts, w, layer in preroute(data):
        board.net_id(net)
        for a, b in zip(pts, pts[1:]):
            board.add_track(net, layer, a, b, w)
    log, failed = [], []
    for net in order:
        w = WIDTH.get(net, DEFAULT_W)
        t0 = time.time()
        ok = route_net(board, net, w, net in allow_f_for, log)
        if VERBOSE:
            print("    %-12s %s %.1fs" % (net, "ok" if ok else "FAIL", time.time() - t0), flush=True)
        if not ok and net in FALLBACK:
            # rip this net's copper and retry narrower
            nid = board.nets[net]
            board.tracks = [t for t in board.tracks if t["net"] != net]
            for layer in (B, F):
                board.label[layer][board.label[layer] == nid] = 0
            for p in board.pads[net]:
                for layer in (B, F):
                    if ("B" if layer == B else "F") in p["layers"]:
                        board.label[layer][pad_mask(p)] = nid
            log.append("  %s: retry at %.2f mm" % (net, FALLBACK[net]))
            ok = route_net(board, net, FALLBACK[net], net in allow_f_for, log)
        if not ok:
            failed.append(net)
    return board, failed, log


def main(src, dst):
    data = json.load(open(src))
    board0 = Board(json.loads(json.dumps(data)))
    order = order_nets(board0)
    # VBAT and GND reach every corner: they may use top tracks from the start (MILLING.md)
    allow_f = set(os.environ.get("ROUTER_JUMPERS", "GND,VBAT").split(",")) - {""}
    best = None
    for rnd in range(ROUNDS):
        board, failed, log = run(data, order, allow_f)
        print("round %d: %d nets failed %s (jumpers allowed: %s)" % (rnd, len(failed), failed, sorted(allow_f) or "none"),
              flush=True)
        for line in log:
            print(line)
        score = (len(failed), len(board.vias))
        if best is None or score < best[0]:
            best = (score, board, failed, list(order))
        if not failed:
            break
        order = failed + [n for n in order if n not in failed]
        if rnd >= ROUNDS // 2:  # second half: let the stubborn nets use top-side wire jumpers
            allow_f |= set(failed)
    _, board, failed, order = best
    with open(dst, "w") as f:
        json.dump({"tracks": board.tracks, "vias": board.vias, "failed": failed, "order": order}, f, indent=1)
    jumpers = [t for t in board.tracks if t["layer"] == "F.Cu"]
    print("wrote %s: %d tracks (%d top-side jumper segments), %d wire vias, failed: %s"
          % (dst, len(board.tracks), len(jumpers), len(board.vias), failed or "none"))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))

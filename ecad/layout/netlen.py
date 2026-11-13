#!/usr/bin/env python3
"""Pad-to-pad copper length of a net, from the geometry dump (layout/dump.py).

    lengths("QSPI_CLK", data, ("MCU.U1.X", None), [("FLASH.U5.X", None)], z)

The net's tracks and vias become a graph: track ends are nodes, a track is an edge of its
length, a track end that lands in the middle of another track splits it (T junction), a via
joins the layers it crosses with an edge as long as the distance between those copper layers
(z: layer name -> mm, from impedance.copper_spans), and a pad joins every track end inside it
with an edge to the pad centre.  The result is the shortest path from the source pad to each
target pad, so stubs (a pull-up resistor, the ESD array's tap) are not counted.

Pure Python, shared by check_layout.py and autoroute.py.
"""
import heapq
import math

SNAP = 0.006   # mm: coordinates are rounded to 0.1 um in the dump; ends closer than this are one point
LAYER = {"F": "F.Cu", "B": "B.Cu", "I": "In2.Cu"}


def _key(layer, p):
    return (layer, round(p[0] / SNAP), round(p[1] / SNAP))


def _on_segment(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L2 = vx * vx + vy * vy
    if L2 == 0:
        return None
    t = ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2
    if t <= 0 or t >= 1:
        return None
    if math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy) > SNAP:
        return None
    return t


def graph(net, tracks, vias, pads, z):
    """pads: [(name, layer names, centre, box, round)] of this net."""
    tr = [(LAYER.get(t["layer"], t["layer"]), tuple(t["start"]), tuple(t["end"])) for t in tracks if t["net"] == net]
    adj = {}

    def link(a, b, w):
        adj.setdefault(a, []).append((b, w))
        adj.setdefault(b, []).append((a, w))

    ends = {}
    for layer, a, b in tr:
        for p in (a, b):
            ends.setdefault(layer, {})[_key(layer, p)] = p
    for layer, a, b in tr:
        cuts = [(0.0, a), (1.0, b)]
        for k, p in ends[layer].items():
            t = _on_segment(p, a, b)
            if t is not None:
                cuts.append((t, p))
        cuts.sort()
        for (_, p), (_, q) in zip(cuts, cuts[1:]):
            link(_key(layer, p), _key(layer, q), math.dist(p, q))
    for v in vias:
        if v["net"] != net:
            continue
        c = tuple(v["at"])
        r = v["dia"] / 2
        nodes = []
        for layer in z:
            node = ("via", layer, round(c[0] / SNAP), round(c[1] / SNAP))
            nodes.append((layer, node))
            for k, p in ends.get(layer, {}).items():
                d = math.dist(p, c)
                if d <= r:
                    link(node, k, d)
            for la, a, b in tr:      # a track that runs through the via without ending there
                if la == layer and _on_segment(c, a, b) is not None:
                    link(node, _key(layer, a), math.dist(c, a))
                    link(node, _key(layer, b), math.dist(c, b))
        for (la, na), (lb, nb) in zip(nodes, nodes[1:]):
            link(na, nb, abs(z[la] - z[lb]))
    for name, layers, centre, box, _ in pads:
        node = ("pad", name)
        adj.setdefault(node, [])
        for layer in layers:
            for k, p in ends.get(layer, {}).items():
                if box[0] - SNAP <= p[0] <= box[2] + SNAP and box[1] - SNAP <= p[1] <= box[3] + SNAP:
                    link(node, k, math.dist(p, centre))
            for v in vias:           # a via in the pad (thermal pads)
                if v["net"] == net and box[0] <= v["at"][0] <= box[2] and box[1] <= v["at"][1] <= box[3]:
                    link(node, ("via", layer, round(v["at"][0] / SNAP), round(v["at"][1] / SNAP)), math.dist(v["at"], centre))
    return adj


def shortest(adj, src):
    dist = {src: 0.0}
    heap = [(0.0, 0, src)]
    n = 0
    while heap:
        d, _, u = heapq.heappop(heap)
        if d > dist.get(u, 1e18):
            continue
        for v, w in adj.get(u, ()):
            nd = d + w
            if nd < dist.get(v, 1e18):
                dist[v] = nd
                n += 1
                heapq.heappush(heap, (nd, n, v))
    return dist


def net_pads(data, net):
    out = []
    for f in data["footprints"]:
        for p in f["pads"]:
            if p["net"] == net and (p["layers"] or p["drill"]):
                layers = [LAYER[x] for x in p["layers"]] if not p["drill"] else list(LAYER.values())
                out.append(((f["path"], p["number"]), layers, tuple(p["at"]), p["box"], p.get("round")))
    return out


def lengths(net, data, src, dsts, z):
    """Shortest copper length from pad src=(path, number or None) to each pad in dsts; None if not connected."""
    pads = net_pads(data, net)

    def find(sel):
        hit = [p[0] for p in pads if p[0][0] == sel[0] and (sel[1] is None or p[0][1] == sel[1])]
        assert hit, (net, sel)
        return hit[0]

    adj = graph(net, data["tracks"], data["vias"], pads, z)
    dist = shortest(adj, ("pad", find(src)))
    return {d: dist.get(("pad", find(d))) for d in dsts}

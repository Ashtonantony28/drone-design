#!/usr/bin/env python3
"""Review pictures of the layout from layout/dump.py output (host Python + matplotlib).

    python3 layout/plot.py <board.json> <out_prefix> [drc.json]
        -> <out_prefix>_top.png     top side: through-hole parts, outlines, top jumpers (if any)
        -> <out_prefix>_bottom.png  bottom copper and SMD parts, seen from the top (x not mirrored)
        -> <out_prefix>_mill.png    bottom copper seen from the BOTTOM (mirrored), as it goes on the mill

Unrouted connections (ratsnest from pad centres, or the DRC report's
unconnected items) are drawn in green.
"""
import json
import math
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Polygon, Rectangle  # noqa: E402

CU = {"F.Cu": "#c83232", "B.Cu": "#2a6fd6"}


def pad_patch(p, color, alpha=0.85):
    (x, y), (w, h) = p["at"], p["size"]
    a = math.radians(p.get("angle", 0))
    if p.get("shape") == "circle":
        return Circle((x, y), w / 2, fc=color, ec="none", alpha=alpha)
    if abs(math.sin(a)) > 0.7:  # quarter turn
        w, h = h, w
    if p.get("shape") == "oval":
        r = min(w, h) / 2
        return FancyBboxPatch((x - w / 2 + r, y - h / 2 + r), w - 2 * r, h - 2 * r, boxstyle="round,pad=%g" % r,
                              fc=color, ec="none", alpha=alpha)
    return Rectangle((x - w / 2, y - h / 2), w, h, fc=color, ec="none", alpha=alpha)


def outline(ax, d):
    for e in d["edges"]:
        if e["center"]:
            (cx, cy), (sx, sy), (ex, ey) = e["center"], e["start"], e["end"]
            r = math.hypot(sx - cx, sy - cy)
            a0, a1 = math.atan2(sy - cy, sx - cx), math.atan2(ey - cy, ex - cx)
            if a1 < a0:
                a1 += 2 * math.pi
            if a1 - a0 > math.pi:
                a0, a1 = a1, a0 + 2 * math.pi
            ts = [a0 + (a1 - a0) * i / 16 for i in range(17)]
            ax.plot([cx + r * math.cos(t) for t in ts], [cy + r * math.sin(t) for t in ts], "k", lw=1.2)
        else:
            ax.plot([e["start"][0], e["end"][0]], [e["start"][1], e["end"][1]], "k", lw=1.2)


def ratsnest(d):
    """Pad-centre minimum spanning tree per net, minus nets the tracks already join (rough)."""
    nets = {}
    for fp in d["footprints"]:
        for p in fp["pads"]:
            if p["net"] and not p["net"].startswith(("U1.NC", "U2.NC", "U4.NC", "SW1.NC")):
                nets.setdefault(p["net"], []).append(tuple(p["at"]))
    routed = {t["net"] for t in d["tracks"]}
    lines = []
    for net, pts in nets.items():
        if net in routed or len(pts) < 2:
            continue
        done, todo = [pts[0]], pts[1:]
        while todo:
            a, b = min(((a, b) for a in done for b in todo), key=lambda ab: math.dist(*ab))
            lines.append((a, b))
            done.append(b)
            todo.remove(b)
    return lines


def draw(d, side, path, rats, mirror=False):
    fig, ax = plt.subplots(figsize=(12, 10), dpi=110)
    outline(ax, d)
    for z in d["zones"]:
        if not z["keepout"] and ("B.Cu" in z["layers"]) == (side == "B"):
            ax.add_patch(Polygon(z["outline"], closed=True, fc=CU["B.Cu"], alpha=0.07, ec="none"))
    for t in d["tracks"]:
        mine = (t["layer"] == "B.Cu") == (side == "B")
        ax.plot([t["start"][0], t["end"][0]], [t["start"][1], t["end"][1]], color=CU[t["layer"]],
                lw=t["width"] * 7.5, alpha=0.9 if mine else 0.15, solid_capstyle="round")
    for v in d["vias"]:
        ax.add_patch(Circle(v["at"], v["dia"] / 2, fc="#888", ec="k", lw=0.3))
    for fp in d["footprints"]:
        mine = fp["side"] == side
        c = fp["courtyard"]
        if c:
            ax.add_patch(Rectangle((c[0], c[1]), c[2] - c[0], c[3] - c[1], fill=False,
                                   ec="#b000b0" if fp["side"] == "F" else "#008080", lw=0.8 if mine else 0.3,
                                   ls="-" if mine else ":"))
            if mine:
                ax.text((c[0] + c[2]) / 2, c[1] - 0.3, fp["ref"], ha="center", va="bottom", fontsize=6)
        for p in fp["pads"]:
            through = p["drill"] > 0
            if not (mine or through):
                continue
            col = "#bbbbbb" if p["npth"] else ("#e09000" if through else CU["B.Cu" if fp["side"] == "B" else "F.Cu"])
            ax.add_patch(pad_patch(p, col))
            if through:
                ax.add_patch(Circle(p["at"], p["drill"] / 2, fc="white", ec="none"))
            if p["net"] and not p["npth"]:
                ax.text(p["at"][0], p["at"][1], p["net"].replace("U1.NC_X_", "nc").replace("U2.NC_X_", "nc")[:7],
                        ha="center", va="center", fontsize=3.2, color="k")
    for a, b in rats:
        ax.plot([a[0], b[0]], [a[1], b[1]], color="#00b000", lw=0.7)
    x0, y0, x1, y1 = d["edge"]
    ax.set_xlim((x1 + 1, x0 - 1) if mirror else (x0 - 1, x1 + 1))
    ax.set_ylim(y1 + 1, y0 - 1)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.2)
    ax.set_xticks(range(-22, 23, 2))
    ax.set_yticks(range(-18, 19, 2))
    title = {"F": "top side (through-hole parts)", "B": "bottom copper + SMD, seen from the top"}[side]
    if mirror:
        title = "bottom copper as it goes on the mill (seen from below, mirrored)"
    ax.set_title(title + "  [mm from board centre, nose = +x]")
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    data = json.load(open(sys.argv[1]))
    if len(sys.argv) > 3:
        rats = []
        for u in json.load(open(sys.argv[3])).get("unconnected_items", []):
            a, b = u["items"][0]["pos"], u["items"][-1]["pos"]
            rats.append(((a["x"] - 100, a["y"] - 100), (b["x"] - 100, b["y"] - 100)))
    else:
        rats = ratsnest(data)
    draw(data, "F", sys.argv[2] + "_top.png", rats)
    draw(data, "B", sys.argv[2] + "_bottom.png", rats)
    draw(data, "B", sys.argv[2] + "_mill.png", [], mirror=True)

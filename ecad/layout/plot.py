#!/usr/bin/env python3
"""Draw review pictures of the layout from layout/dump.py output (host Python + matplotlib).

    python3 layout/plot.py <dump.json> <out_prefix> [drc.json]   -> <out_prefix>_top.png, _bottom.png

Both pictures use top-view coordinates (the bottom side is seen through the board).
"""
import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

LAYER_COLOR = {"F.Cu": "#c83232", "B.Cu": "#3250c8", "In1.Cu": "#32a032", "In2.Cu": "#c8a000"}


def draw(d, side, path, labels=True, rats=()):
    fig, ax = plt.subplots(figsize=(14, 14), dpi=110)
    x0, y0, x1, y1 = d["edge"]
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="k", lw=1.5))
    for z in d["zones"]:
        if z["keepout"]:
            xs = [p[0] for p in z["outline"]] + [z["outline"][0][0]]
            ys = [p[1] for p in z["outline"]] + [z["outline"][0][1]]
            ax.plot(xs, ys, color="#888", lw=0.6, ls="--")
    cu = "F.Cu" if side == "F" else "B.Cu"
    for t in d["tracks"]:
        if t["layer"] in (cu, "In1.Cu", "In2.Cu"):
            ax.plot([t["start"][0], t["end"][0]], [t["start"][1], t["end"][1]], color=LAYER_COLOR[t["layer"]],
                    lw=max(0.6, t["width"] * 4), alpha=0.9 if t["layer"] == cu else 0.35, solid_capstyle="round")
    for v in d["vias"]:
        ax.add_patch(Circle(v["at"], v["dia"] / 2, fc="#666", ec="k", lw=0.2, alpha=0.7))
    for fp in d["footprints"]:
        mine = fp["side"] == side
        c = fp["courtyard"]
        if c and mine:
            ax.add_patch(Rectangle((c[0], c[1]), c[2] - c[0], c[3] - c[1], fill=False, ec="#b000b0" if not fp["dnp"] else "#999", lw=0.6))
            if labels:
                ax.text((c[0] + c[2]) / 2, (c[1] + c[3]) / 2, fp["ref"], ha="center", va="center", fontsize=5, color="#000")
        for p in fp["pads"]:
            through = p["drill"] > 0
            if not (mine or through):
                continue
            b = p["box"]
            col = "#aaa" if p["npth"] else ("#e09000" if through else ("#c83232" if side == "F" else "#3250c8"))
            ax.add_patch(Rectangle((b[0], b[1]), b[2] - b[0], b[3] - b[1], fc=col, ec="none", alpha=0.75))
    for a, b in rats:
        ax.plot([a[0], b[0]], [a[1], b[1]], color="#00a000", lw=0.8)
    ax.set_xlim(x0 - 1, x1 + 1)
    ax.set_ylim(y1 + 1, y0 - 1)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.2)
    ax.set_xticks(range(-18, 19, 2))
    ax.set_yticks(range(-18, 19, 2))
    ax.set_title("%s side (top-view coordinates, mm)" % ("top" if side == "F" else "bottom"))
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    data = json.load(open(sys.argv[1]))
    rats = []
    if len(sys.argv) > 3:  # kicad-cli DRC report: draw what is still unconnected in green
        ox = (data["edge"][0] + data["edge"][2]) / 2
        for u in json.load(open(sys.argv[3])).get("unconnected_items", []):
            a, b = u["items"][0]["pos"], u["items"][-1]["pos"]
            rats.append(((a["x"] - 100, a["y"] - 100), (b["x"] - 100, b["y"] - 100)))
    draw(data, "F", sys.argv[2] + "_top.png", rats=rats)
    draw(data, "B", sys.argv[2] + "_bottom.png", rats=rats)

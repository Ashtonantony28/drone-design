"""Manufacturing and review outputs for the NF-3D airframe.

    python -m mech.export
writes to out/cad/:
  nf3d_assembly.step          whole assembly (frame + envelopes of bought parts)
  bottom_plate.step/.dxf      3.0 mm carbon plate: STEP plus 2-D DXF for CNC/waterjet
  top_plate.step/.dxf         1.5 mm carbon plate
  duct.stl                    print one per motor (PA12-CF, MJF/SLS or FDM)
  foot.stl                    TPU landing foot, x4
  render_iso.png, render_top.png, render_side.png   matplotlib renders (no GPU needed)
"""
from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from build123d import Axis, Compound, ExportDXF, Pos, export_step, export_stl  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

from .frame import bottom_plate, build_assembly, duct, foot, load_spec, top_plate  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "out", "cad")

COLORS = {"frame": "#3b3b3b", "propulsion": "#c0392b", "avionics": "#2e86c1", "battery": "#d4ac0d"}
DUCT_COLOR = "#7f8c8d"


def plate_dxf(part, path, z):
    sec = part.faces().sort_by(Axis.Z)[0]
    sec = Pos(0, 0, -sec.center().Z) * sec  # DXF is 2-D: drop the face onto z = 0
    exp = ExportDXF()
    exp.add_layer("cut")
    exp.add_shape(sec, layer="cut")
    exp.write(path)


def tris(part, tol=0.2):
    verts, faces = part.tessellate(tol)
    v = np.array([[p.X, p.Y, p.Z] for p in verts])
    return v[np.array(faces)]


def render(comps, path, elev, azim, title):
    fig = plt.figure(figsize=(8, 7), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    light = np.array([0.4, 0.3, 0.85]); light /= np.linalg.norm(light)
    for c in comps:
        t = tris(c.part, 0.3)
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.45 + 0.55 * np.abs(n @ light)
        base = DUCT_COLOR if c.name.startswith("duct") else COLORS.get(c.group, "#555")
        rgb = np.array(matplotlib.colors.to_rgb(base))
        fc = np.clip(rgb[None, :] * shade[:, None], 0, 1)
        alpha = 0.55 if c.name.startswith("duct") else 1.0
        pc = Poly3DCollection(t, facecolors=fc, edgecolors="none", alpha=alpha)
        ax.add_collection3d(pc)
    lim = 100
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim / 2, lim / 2 + 20)
    ax.set_box_aspect((2, 2, 1.2))
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("x fwd (mm)"); ax.set_ylabel("y left (mm)"); ax.set_zlabel("z (mm)")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    spec = load_spec()
    comps = build_assembly(spec)
    export_step(Compound([c.part for c in comps]), os.path.join(OUT, "nf3d_assembly.step"))
    bp, tp = bottom_plate(spec), top_plate(spec)
    export_step(bp, os.path.join(OUT, "bottom_plate.step"))
    export_step(tp, os.path.join(OUT, "top_plate.step"))
    plate_dxf(bp, os.path.join(OUT, "bottom_plate.dxf"), 0.0)
    plate_dxf(tp, os.path.join(OUT, "top_plate.dxf"), 0.0)
    export_stl(duct(spec, 0, 0), os.path.join(OUT, "duct.stl"))
    export_stl(foot(spec, 0, 0), os.path.join(OUT, "foot.stl"))
    render(comps, os.path.join(OUT, "render_iso.png"), 28, -60, "NF-3D assembly (iso)")
    render(comps, os.path.join(OUT, "render_top.png"), 90, -90, "NF-3D top view (x forward to the right)")
    render(comps, os.path.join(OUT, "render_side.png"), 0, -90, "NF-3D side view")
    for f in sorted(os.listdir(OUT)):
        print(f"{f:28s} {os.path.getsize(os.path.join(OUT, f))/1024:8.1f} KB")


if __name__ == "__main__":
    main()

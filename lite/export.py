"""NF-3D Lite manufacturing and review outputs.

    python -m lite.export
writes lite/out/cad/:
  frame.stl / frame.step     the one-piece PETG frame (print flat, plate side down, 100 % infill on the arms)
  guard.stl                  the optional clip-on prop guard (print flat)
  assembly.step              frame + envelopes of the bought parts, for checking in Fusion / FreeCAD
  detail.step                the detailed visual model (lite/detail.py)
  viewer_model.json          tessellated detailed model for the browser viewer
"""
from __future__ import annotations

import base64
import json
import os

import numpy as np
from build123d import Compound, export_step, export_stl

from .detail import build_detail
from .frame import build_assembly, frame, guard, load_spec

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "cad")


def main():
    os.makedirs(OUT, exist_ok=True)
    spec = load_spec()
    fr = frame(spec)
    export_stl(fr, os.path.join(OUT, "frame.stl"), tolerance=0.02, angular_tolerance=0.1)
    export_step(fr, os.path.join(OUT, "frame.step"))
    export_stl(guard(spec), os.path.join(OUT, "guard.stl"), tolerance=0.02, angular_tolerance=0.1)
    comps = build_assembly(spec)
    export_step(Compound([c.part for c in comps]), os.path.join(OUT, "assembly.step"))
    detail = build_detail(spec)
    export_step(Compound([p.part for p in detail if p.group != "sweep"]), os.path.join(OUT, "detail.step"))
    parts = []
    for p in detail:
        # curved bought parts and the frame fine; lofted props and flat boxes coarser (file size)
        tol = (0.08, 0.3) if p.name.startswith("Prop") else (0.02, 0.12) if p.group in ("propulsion", "frame", "guard", "battery") else (0.05, 0.3)
        v, f = p.part.tessellate(*tol)
        V = np.round(np.array([[q.X, q.Y, q.Z] for q in v], dtype=np.float32), 3)
        F = np.array(f, dtype=np.uint32).reshape(-1)
        parts.append({"name": p.name, "group": p.group, "color": p.color, "metal": p.metal, "rough": p.rough,
                      "opacity": p.opacity, "pos": base64.b64encode(V.astype(np.float32).tobytes()).decode(),
                      "idx": base64.b64encode(F.tobytes()).decode()})
    with open(os.path.join(OUT, "viewer_model.json"), "w") as fh:
        json.dump(parts, fh)
    for n in sorted(os.listdir(OUT)):
        print(f"{n:22s} {os.path.getsize(os.path.join(OUT, n)) / 1024:8.1f} KB")


if __name__ == "__main__":
    main()

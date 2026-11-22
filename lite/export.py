"""NF-3D Lite manufacturing and review outputs.

    python -m lite.export
writes lite/out/cad/:
  frame.stl / frame.step     the one-piece PLA frame (print flat, plate side down, 100 % infill on the arms)
  assembly.step              frame + envelopes of the bought parts, for checking in Fusion / FreeCAD
  viewer_model.json          tessellated assembly for the browser viewer
"""
from __future__ import annotations

import base64
import json
import os

import numpy as np
from build123d import Compound, export_step, export_stl

from .frame import build_assembly, frame, load_spec, motor_xy, prop_disk

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "cad")


def main():
    os.makedirs(OUT, exist_ok=True)
    spec = load_spec()
    fr = frame(spec)
    export_stl(fr, os.path.join(OUT, "frame.stl"), tolerance=0.02, angular_tolerance=0.1)
    export_step(fr, os.path.join(OUT, "frame.step"))
    comps = build_assembly(spec)
    export_step(Compound([c.part for c in comps]), os.path.join(OUT, "assembly.step"))
    # the viewer also shows the swept prop discs (not physical parts, so not in the STEP)
    shown = [(c.name, c.group, c.part) for c in comps]
    shown += [(f"prop_{i}", "prop", prop_disk(spec, x, y)) for i, (x, y) in enumerate(motor_xy(spec))]
    parts = []
    for name, group, part in shown:
        v, f = part.tessellate(0.1, 0.25)
        V = np.array([[p.X, p.Y, p.Z] for p in v], dtype=np.float32)
        F = np.array(f, dtype=np.uint32).reshape(-1)
        parts.append({"name": name, "group": group, "pos": base64.b64encode(V.tobytes()).decode(),
                      "idx": base64.b64encode(F.tobytes()).decode()})
    with open(os.path.join(OUT, "viewer_model.json"), "w") as fh:
        json.dump(parts, fh)
    for n in sorted(os.listdir(OUT)):
        print(f"{n:22s} {os.path.getsize(os.path.join(OUT, n)) / 1024:8.1f} KB")


if __name__ == "__main__":
    main()

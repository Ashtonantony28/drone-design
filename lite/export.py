"""NF-3D Lite manufacturing and review outputs.

    python -m lite.export
writes lite/out/cad/:
  frame.stl / frame.step     the one-piece PETG frame (print flat, plate side down, 100 % infill on the arms)
  guard.stl                  the optional clip-on prop guard (print flat)
  assembly.step              frame + envelopes of the bought parts, for checking in Fusion / FreeCAD
  detail.step                the detailed visual model (lite/detail.py)
  viewer_model.json.gz       tessellated detailed model for the browser viewer (16-bit quantised)
"""
from __future__ import annotations

import base64
import gzip
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
        # round parts read as round (angular tolerance 0.1 rad, about 6 degrees; props 0.15) while
        # the viewer file stays under the 16 MB page limit
        tol = (0.1, 0.15) if p.name.startswith("Prop") else (0.04, 0.1)
        v, f = p.part.tessellate(*tol)
        V = np.array([[q.X, q.Y, q.Z] for q in v], dtype=np.float64)
        F = np.array(f, dtype=np.uint32).reshape(-1)
        # compact: positions quantised to 16 bits over the part's own box (under 2.5 um per step for
        # the largest part), indices 16-bit where they fit; the viewer undoes both
        lo = V.min(axis=0)
        step = float(max((V.max(axis=0) - lo).max(), 1e-6) / 65535.0)
        Q = np.round((V - lo) / step).astype(np.uint16)
        I = F.astype(np.uint16) if len(V) < 65536 else F
        parts.append({"name": p.name, "group": p.group, "color": p.color, "metal": p.metal, "rough": p.rough,
                      "opacity": p.opacity, "lo": [round(float(c), 5) for c in lo], "step": step,
                      "pos": base64.b64encode(Q.tobytes()).decode(),
                      "idx": base64.b64encode(I.tobytes()).decode(), "i16": bool(len(V) < 65536)})
    blob = json.dumps(parts).encode()
    with gzip.open(os.path.join(OUT, "viewer_model.json.gz"), "wb", compresslevel=9) as fh:
        fh.write(blob)
    for n in sorted(os.listdir(OUT)):
        print(f"{n:22s} {os.path.getsize(os.path.join(OUT, n)) / 1024:8.1f} KB")


if __name__ == "__main__":
    main()

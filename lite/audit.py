"""Geometry audit of the NF-3D Lite detailed model (lite/detail.py).

    python -m lite.audit [--no-guard]        -> lite/out/audit.json, exits 1 on any failure

Checks, on the exact B-rep solids (not the viewer mesh):
  clash     every part against every other part: intersection volume. Any overlap above
            0.001 mm^3 fails unless it is listed in ALLOWED with the build reason.
  contact   which parts touch (B-rep distance <= 0.02 mm). Every part must be connected to
            the frame through a chain of touching parts: no floating parts.
  swing     each prop's swept disc (plus 1 mm) against everything that is not that prop,
            its shaft or its own hub: minimum clearance.
  solids    every part is one valid closed solid with positive volume.
  boxes     parts that are still a bare six-faced box (a placeholder look) are listed.
"""
from __future__ import annotations

import argparse
import itertools
import json
import os
import re

from build123d import Part

from .detail import build_detail
from .frame import load_spec, motor_xy, prop_disk

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

CLASH_TOL = 1e-3     # mm^3
TOUCH_TOL = 0.02     # mm
SWING_MIN = load_spec()["geometry_mm"]["prop_tip_gap"]   # mm, prop disc to anything not on its motor (v4: 3.0, review L10)
SWING_OWN = 0.95     # mm, prop disc to its own motor's bearing boss (the designed 1.0 mm shaft stub gap)

# (regex for part A, regex for part B, build reason). Matched in both orders.
ALLOWED: list[tuple[str, str, str]] = []


def _bb(p: Part):
    b = p.bounding_box()
    return (b.min.X, b.min.Y, b.min.Z, b.max.X, b.max.Y, b.max.Z)


def _near(a, b, pad):
    return all(a[i] - pad <= b[i + 3] and b[i] - pad <= a[i + 3] for i in range(3))


def _allowed(na, nb):
    for ra, rb, why in ALLOWED:
        if (re.search(ra, na) and re.search(rb, nb)) or (re.search(ra, nb) and re.search(rb, na)):
            return why
    return None


def _is_box(p: Part):
    fs = p.faces()
    return len(p.solids()) == 1 and len(fs) == 6 and all("PLANE" in str(f.geom_type) for f in fs)


def audit(with_guard=True, verbose=True):
    spec = load_spec()
    parts = [p for p in build_detail(spec, with_guard=with_guard) if p.group != "sweep"]
    names = [f"{k:03d} {p.name}" for k, p in enumerate(parts)]
    bbs = [_bb(p.part) for p in parts]
    rep = {"parts": len(parts), "clashes": [], "allowed": [], "invalid": [], "boxes": [], "floating": [],
           "swing": [], "contacts": 0}

    # solids
    for n, p in zip(names, parts):
        s = p.part
        if not s.is_valid or s.volume <= 0 or len(s.solids()) == 0:
            rep["invalid"].append(n)
        if _is_box(s):
            rep["boxes"].append(n)

    # clash + contact
    adj = {i: set() for i in range(len(parts))}
    for i, j in itertools.combinations(range(len(parts)), 2):
        if not _near(bbs[i], bbs[j], TOUCH_TOL):
            continue
        a, b = parts[i].part, parts[j].part
        try:
            d = a.distance_to(b)
        except Exception:
            d = 0.0
        if d > TOUCH_TOL:
            continue
        adj[i].add(j); adj[j].add(i)
        try:
            v = (a & b).volume
        except Exception:
            v = float("nan")
        if not v <= CLASH_TOL:
            why = _allowed(names[i], names[j])
            row = {"a": names[i], "b": names[j], "mm3": round(v, 4)}
            (rep["allowed"] if why else rep["clashes"]).append({**row, "reason": why} if why else row)
    rep["contacts"] = sum(len(s) for s in adj.values()) // 2

    # floating: connected component of the frame
    root = next(k for k, p in enumerate(parts) if p.group == "frame")
    seen, stack = {root}, [root]
    while stack:
        k = stack.pop()
        for m in adj[k] - seen:
            seen.add(m); stack.append(m)
    rep["floating"] = [names[k] for k in range(len(parts)) if k not in seen]

    # prop swing
    for i, (x, y) in enumerate(motor_xy(spec)):
        disc = prop_disk(spec, x, y)
        bd = _bb(disc)
        worst, other = (1e9, None), (1e9, None)   # anything / anything not on this motor
        for k, p in enumerate(parts):
            if p.name.startswith(f"Prop {i} ") or p.name.startswith(f"Motor {i} shaft"):
                continue
            if not _near(bd, bbs[k], SWING_MIN + 15):
                continue
            d = disc.distance_to(p.part)
            if d < worst[0]:
                worst = (d, names[k])
            if not p.name.startswith(f"Motor {i} ") and d < other[0]:
                other = (d, names[k])
        rep["swing"].append({"prop": i, "min_clearance_mm": round(worst[0], 3), "nearest": worst[1],
                             "min_clearance_off_motor_mm": round(other[0], 3), "nearest_off_motor": other[1]})
    swing_fail = [s for s in rep["swing"] if s["min_clearance_off_motor_mm"] < SWING_MIN or s["min_clearance_mm"] < SWING_OWN]

    ok = not (rep["clashes"] or rep["invalid"] or rep["floating"] or swing_fail)
    rep["result"] = "PASS" if ok else "FAIL"
    if verbose:
        print(f"{rep['parts']} parts, {rep['contacts']} touching pairs")
        print(f"clashes: {len(rep['clashes'])}  (allowed with a reason: {len(rep['allowed'])})")
        for c in rep["clashes"]:
            print(f"  CLASH {c['mm3']:9.4f} mm^3  {c['a']}  x  {c['b']}")
        for c in rep["allowed"]:
            print(f"  ok    {c['mm3']:9.4f} mm^3  {c['a']}  x  {c['b']}  ({c['reason']})")
        print(f"floating: {len(rep['floating'])}")
        for n in rep["floating"]:
            print(f"  FLOAT {n}")
        print(f"invalid solids: {len(rep['invalid'])}", *rep["invalid"], sep="\n  ")
        for s in rep["swing"]:
            print(f"  prop {s['prop']} swing clearance {s['min_clearance_mm']:.2f} mm to {s['nearest']}; "
                  f"off its motor {s['min_clearance_off_motor_mm']:.2f} mm to {s['nearest_off_motor']}")
        print(f"bare boxes (placeholder look): {len(rep['boxes'])}")
        for n in rep["boxes"]:
            print(f"  BOX {n}")
        print(rep["result"])
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-guard", action="store_true")
    a = ap.parse_args(argv)
    rep = audit(not a.no_guard)
    with open(os.path.join(OUT, "audit.json"), "w") as f:
        json.dump(rep, f, indent=1)
    raise SystemExit(0 if rep["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()

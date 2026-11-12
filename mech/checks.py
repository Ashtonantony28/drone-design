"""Geometry and mass-property checks on the NF-3D assembly.

`python -m mech.checks` prints a report and exits non-zero on any failure.
Checks:
  1. interference: no two solids overlap (intersection volume < 1e-3 mm^3)
  2. clearance: parts that are not mounted against each other keep >= min_clearance
  3. prop clearance: each swept prop disk clears every solid except its own motor
     by >= prop_tip_gap - 0.05 mm (radial tip gap to the duct)
  4. landing: the lowest points are the feet, not the flow sensor
  5. mass properties: total mass, CoG, inertia tensor about CoG
"""
from __future__ import annotations

import itertools
import json
import os
import sys

import numpy as np
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from .frame import Component, build_assembly, dims, load_spec, lumped_masses, prop_disk, motor_xy

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def min_distance(a, b) -> float:
    ext = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    ext.Perform()
    return ext.Value()


def overlap_volume(a, b) -> float:
    bb_a, bb_b = a.bounding_box(), b.bounding_box()
    if (bb_a.max.X < bb_b.min.X or bb_b.max.X < bb_a.min.X or bb_a.max.Y < bb_b.min.Y
            or bb_b.max.Y < bb_a.min.Y or bb_a.max.Z < bb_b.min.Z or bb_b.max.Z < bb_a.min.Z):
        return 0.0
    inter = a & b
    return float(inter.volume) if inter is not None else 0.0


def solid_props(part):
    """Volume (mm^3), centroid (mm), inertia matrix about centroid for unit density (mm^5)."""
    gp = GProp_GProps()
    BRepGProp.VolumeProperties_s(part.wrapped, gp)
    c = gp.CentreOfMass()
    m = gp.MatrixOfInertia()
    I = np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return gp.Mass(), np.array([c.X(), c.Y(), c.Z()]), I


def mass_properties(spec: dict, comps: list[Component]) -> dict:
    """Total mass (kg), CoG (m), inertia about CoG (kg m^2), and a per-item table."""
    items = []  # (name, mass_kg, centroid_m, I_about_own_centroid_kgm2)
    for c in comps:
        vol, cen, I_unit = solid_props(c.part)
        if c.mass_g is not None:
            m = c.mass_g / 1000.0
            rho = m / (vol * 1e-9)
        else:
            rho = spec["materials"][c.material]["density"]
            m = rho * vol * 1e-9
        items.append((c.name, m, cen * 1e-3, I_unit * rho * 1e-15))
    for name, mg, pos in lumped_masses(spec):
        items.append((name, mg / 1000.0, np.array(pos) * 1e-3, np.zeros((3, 3))))
    M = sum(i[1] for i in items)
    cog = sum(i[1] * i[2] for i in items) / M
    I = np.zeros((3, 3))
    for _, m, c, Ic in items:
        r = c - cog
        I += Ic + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))  # parallel axis
    return {"mass_kg": M, "cog_m": cog, "inertia_kgm2": I,
            "items": [(n, m) for n, m, _, _ in items]}


def run_checks(spec: dict | None = None, verbose: bool = True) -> dict:
    spec = spec or load_spec()
    g, d = spec["geometry_mm"], dims(spec)
    comps = build_assembly(spec)
    by_name = {c.name: c for c in comps}
    results = {"failures": []}

    def fail(msg):
        results["failures"].append(msg)

    # 1+2 pairwise
    min_gap = []
    for a, b in itertools.combinations(comps, 2):
        ov = overlap_volume(a.part, b.part)
        if ov > 1e-3:
            fail(f"interference {a.name} x {b.name}: {ov:.2f} mm^3")
            continue
        mounted = b.name in a.touches or a.name in b.touches
        if mounted:
            continue
        dist = min_distance(a.part, b.part)
        min_gap.append((dist, a.name, b.name))
        if dist < g["min_clearance"] - 1e-6:
            fail(f"clearance {a.name} - {b.name}: {dist:.2f} mm < {g['min_clearance']} mm")
    min_gap.sort()
    results["tightest_gaps"] = [(round(x, 2), a, b) for x, a, b in min_gap[:6]]

    # 3 props
    prop_gaps = []
    for i, (x, y) in enumerate(motor_xy(spec)):
        disk = prop_disk(spec, x, y)
        for c in comps:
            if c.name == f"motor_{i}":
                continue
            if overlap_volume(disk, c.part) > 1e-3:
                fail(f"prop {i} sweeps through {c.name}")
                continue
            dist = min_distance(disk, c.part)
            if dist < 5.0:
                prop_gaps.append((round(dist, 2), i, c.name))
            if dist < g["prop_tip_gap"] - 0.05:
                fail(f"prop {i} to {c.name}: {dist:.2f} mm < tip gap {g['prop_tip_gap']} mm")
    results["prop_gaps"] = sorted(prop_gaps)[:6]
    # 3b the duct must cover the prop band axially (otherwise the tip gap is meaningless)
    m = g["prop_axial_margin"]
    results["prop_band_z"] = (d["prop_z0"], d["prop_z1"])
    results["duct_z"] = (d["duct_z0"], d["duct_z1"])
    if d["prop_z1"] + m > d["duct_z1"] or d["prop_z0"] - m < d["duct_z0"]:
        fail(f"prop band z {d['prop_z0']:.1f}-{d['prop_z1']:.1f} not inside duct "
             f"{d['duct_z0']:.1f}-{d['duct_z1']:.1f} with {m} mm margin")

    # 4 landing
    lows = sorted((c.part.bounding_box().min.Z, c.name) for c in comps)
    results["lowest"] = [(round(z, 2), n) for z, n in lows[:5]]
    if not lows[0][1].startswith("foot"):
        fail(f"lowest part is {lows[0][1]}, expected a landing foot")

    # 5 mass
    mp = mass_properties(spec, comps)
    results["mass_g"] = mp["mass_kg"] * 1000
    results["cog_mm"] = (mp["cog_m"] * 1000).tolist()
    results["inertia_kgm2"] = mp["inertia_kgm2"].tolist()
    results["mass_items_g"] = [(n, round(m * 1000, 2)) for n, m in mp["items"]]
    req = spec["requirements"]
    if results["mass_g"] > req["max_auw_g"]:
        fail(f"AUW {results['mass_g']:.1f} g > {req['max_auw_g']} g")
    off = float(np.hypot(*results["cog_mm"][:2]))
    results["cog_offset_xy_mm"] = off
    if off > req["max_cog_offset_xy_mm"]:
        fail(f"CoG xy offset {off:.2f} mm > {req['max_cog_offset_xy_mm']} mm")
    results["arm_xy_m"] = g["arm_xy"] / 1000.0
    results["thrust_plane_above_cog_mm"] = (d["prop_z0"] + d["prop_z1"]) / 2 - results["cog_mm"][2]

    if verbose:
        print(f"AUW {results['mass_g']:.1f} g   CoG {np.round(results['cog_mm'], 2)} mm")
        I = np.array(results["inertia_kgm2"])
        print("Inertia about CoG (kg m^2):\n", np.array2string(I, precision=3))
        print("tightest gaps (mm):", results["tightest_gaps"])
        print("prop gaps (mm):", results["prop_gaps"])
        print("lowest parts (z mm):", results["lowest"])
        print(f"prop band z {results['prop_band_z']}, duct z {results['duct_z']}")
        print("FAIL" if results["failures"] else "PASS", *results["failures"], sep="\n  ")
    return results


def main():
    r = run_checks()
    os.makedirs(os.path.join(ROOT, "out"), exist_ok=True)
    with open(os.path.join(ROOT, "out", "mech_checks.json"), "w") as f:
        json.dump(r, f, indent=1)
    sys.exit(1 if r["failures"] else 0)


if __name__ == "__main__":
    main()

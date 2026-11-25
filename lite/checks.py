"""NF-3D Lite geometry, mass and propulsion checks.

    python -m lite.checks
Writes lite/out/checks.json and exits non-zero on any failure:
  - no interference; >= min_clearance between parts that are not mounted together
  - props clear everything by >= prop_tip_gap, and adjacent props clear each other
  - the flat frame underside is the only thing touching the ground
  - mass, CoG offset, inertia (fed to the flight simulation)
  - propulsion (prop/propulsion.py, nominal and pessimistic corners): T/W, hover throttle, flight time
"""
from __future__ import annotations

import copy
import itertools
import json
import math
import os
import sys

import numpy as np

from mech.checks import min_distance, overlap_volume, solid_props
from prop.propulsion import analyse

from .frame import build_assembly, dims, load_spec, lumped_masses, motor_xy, prop_disk

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")


def mass_properties(spec, comps):
    items = []
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
        I += Ic + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return M, cog, I, [(n, round(m * 1000, 2)) for n, m, _, _ in items]


def run(spec=None, verbose=True):
    spec = spec or load_spec()
    g, d, req = spec["geometry_mm"], dims(spec), spec["requirements"]
    comps = build_assembly(spec)
    res = {"failures": []}
    fail = res["failures"].append

    gaps = []
    for a, b in itertools.combinations(comps, 2):
        ov = overlap_volume(a.part, b.part)
        if ov > 1e-3:
            fail(f"interference {a.name} x {b.name}: {ov:.2f} mm^3")
            continue
        if b.name in a.touches or a.name in b.touches:
            continue
        dist = min_distance(a.part, b.part)
        gaps.append((round(dist, 2), a.name, b.name))
        if dist < g["min_clearance"]:
            fail(f"clearance {a.name} - {b.name}: {dist:.2f} mm")
    res["tightest_gaps"] = sorted(gaps)[:6]

    disks = [prop_disk(spec, x, y) for x, y in motor_xy(spec)]
    pg = []
    for i, disk in enumerate(disks):
        for c in comps:
            if c.name == f"motor_{i}":
                continue
            if overlap_volume(disk, c.part) > 1e-3:
                fail(f"prop {i} sweeps through {c.name}")
                continue
            dist = min_distance(disk, c.part)
            pg.append((round(dist, 2), f"prop_{i}", c.name))
            if dist < g["prop_tip_gap"]:
                fail(f"prop {i} to {c.name}: {dist:.2f} mm")
    a = g["arm_xy"]
    tip_to_tip = 2 * a - g["prop_diameter"]  # adjacent motors are 2a apart
    pg.append((round(tip_to_tip, 2), "prop", "adjacent prop"))
    if tip_to_tip < 2 * g["prop_tip_gap"]:
        fail(f"adjacent props {tip_to_tip:.1f} mm apart")
    res["prop_gaps"] = sorted(pg)[:6]

    lows = sorted((round(c.part.bounding_box().min.Z, 2), c.name) for c in comps)
    res["lowest"] = lows[:3]
    if lows[0][1] != "frame" or lows[0][0] != 0.0 or lows[1][0] <= 0.0:
        fail(f"lowest parts {lows[:2]}: only the flat frame underside (z = 0) may touch the ground")

    M, cog, I, items = mass_properties(spec, comps)
    res["frame_print_g"] = dict(items)["frame"]
    res["mass_g"] = round(M * 1000, 2)
    res["cog_mm"] = (cog * 1000).round(2).tolist()
    res["inertia_kgm2"] = I.tolist()
    res["mass_items_g"] = items
    res["arm_xy_m"] = a / 1000.0
    if res["mass_g"] > req["max_auw_g"]:
        fail(f"AUW {res['mass_g']} g > {req['max_auw_g']} g")
    off = math.hypot(*res["cog_mm"][:2])
    if off > req["max_cog_offset_xy_mm"]:
        fail(f"CoG xy offset {off:.2f} mm")

    # arm strength (beam check: the frame is four cantilevers off a stiff centre plate).
    # 20 g landing on ONE leg; full thrust on one arm. The arm leaves the centre plate where
    # the 45-degree line meets the plate edge.
    mat = spec["materials"][g["frame_material"]]
    r_tip = math.hypot(a, a)
    r_root = min(g["centre_x"], g["centre_y"]) / 2 * math.sqrt(2)
    lever = (r_tip - r_root) * 1e-3
    b, h = g["arm_root_width"] * 1e-3, g["arm_height"] * 1e-3  # tapered arm: root section at the plate edge
    Z = b * h * h / 6
    f_crash = M * 9.81 * 20.0
    s_crash = f_crash * lever / Z
    res["arm"] = {"lever_mm": round(lever * 1e3, 1), "crash_stress_mpa": round(s_crash / 1e6, 1),
                  "crash_sf": round(mat["strength"] / s_crash, 2)}
    if res["arm"]["crash_sf"] < 1.5:
        fail(f"arm crash SF {res['arm']['crash_sf']} < 1.5 (20 g on one leg)")

    res["propulsion"] = {}
    base = spec
    if spec["geometry_mm"].get("autonomy", {}).get("fit"):
        # v3: the companion board streams video over Wi-Fi the whole flight
        base = copy.deepcopy(spec)
        base["electrical"]["avionics_w"] += spec["electrical"].get("avionics_autonomy_w", 0.0)
    for corner in ("nominal", "pessimistic"):
        sp = base
        if corner == "pessimistic":
            sp = copy.deepcopy(base)
            sp["requirements"].update(spec.get("requirements_pessimistic", {}))
        p = analyse(sp, mass_kg=M, corner=corner)
        res["propulsion"][corner] = {k: p[k] for k in ("mass_kg", "thrust_to_weight_nominal", "thrust_to_weight_empty",
                                                       "hover_time_min", "thrust_max_n", "thrust_curve_a",
                                                       "yaw_torque_coeff_m", "motor_tau_hover_s", "failures")}
        res["propulsion"][corner]["hover_throttle"] = p["hover"]["u"]
        res["propulsion"][corner]["wot_motor_current_a"] = p["max_full"]["motor_current_a"]
        res["propulsion"][corner]["thrust_max_g"] = p["thrust_max_n"] / 9.81 * 1000
        for f in p["failures"]:
            # nominal must meet everything; the pessimistic corner is reported as a
            # warning (see requirements_pessimistic in spec.yaml): a weak motor batch is
            # caught by the bench test in BUILD.md before anything is assembled
            (fail if corner == "nominal" else res.setdefault("warnings", []).append)(f"{corner}: {f}")

    # motor acceptance (BUILD.md step 1, review L6/L7): the static full-power thrust each motor must
    # show on the scale for this build to meet min_thrust_to_weight. The pessimistic corner is a
    # motor batch that this bench test rejects, so its warnings are what a skipped test would cost.
    need = req["min_thrust_to_weight"] * res["mass_g"] / 4.0
    pess_g = res["propulsion"]["pessimistic"]["thrust_max_g"]
    res["acceptance"] = {"min_motor_thrust_g": round(need, 1),
                         "nominal_motor_thrust_g": round(res["propulsion"]["nominal"]["thrust_max_g"], 1),
                         "pessimistic_motor_thrust_g": round(pess_g, 1),
                         "pessimistic_rejected": pess_g < need}

    if verbose:
        print(f"AUW {res['mass_g']} g (frame print {res['frame_print_g']} g)  CoG {res['cog_mm']} mm")
        print("tightest gaps:", res["tightest_gaps"])
        print("prop gaps:", res["prop_gaps"])
        print("arm:", res["arm"])
        print("lowest:", res["lowest"])
        for k, p in res["propulsion"].items():
            print(f"{k}: T/W {p['thrust_to_weight_nominal']:.2f}, max {p['thrust_max_g']:.0f} g/motor at "
                  f"{p['wot_motor_current_a']:.2f} A, hover {p['hover_throttle']*100:.0f} %, {p['hover_time_min']:.1f} min")
        acc = res["acceptance"]
        print(f"motor acceptance: >= {acc['min_motor_thrust_g']} g each on the scale (nominal "
              f"{acc['nominal_motor_thrust_g']} g, pessimistic {acc['pessimistic_motor_thrust_g']} g"
              f"{' -> rejected' if acc['pessimistic_rejected'] else ''})")
        for w in res.get("warnings", []):
            print("WARN", w)
        print("FAIL" if res["failures"] else "PASS", *res["failures"], sep="\n  ")
    return res


def main():
    spec = load_spec()
    r = run(spec)
    # the optional clip-on prop guard (spec geometry_mm.guard): same checks with it fitted
    sg = copy.deepcopy(spec)
    sg["geometry_mm"]["guard"]["fit"] = True
    print("\n--- with prop guard ---")
    rg = run(sg)
    # the base v2 build (no autonomy kit) for the motor acceptance table in BUILD.md
    sb = copy.deepcopy(spec)
    sb["geometry_mm"]["autonomy"]["fit"] = False
    print("\n--- base build, no autonomy kit ---")
    rb = run(sb)
    r["base"] = {"mass_g": rb["mass_g"], "acceptance": rb["acceptance"], "failures": rb["failures"],
                 "warnings": rb.get("warnings", []),
                 "propulsion": {k: {kk: v[kk] for kk in ("thrust_to_weight_nominal", "hover_throttle", "hover_time_min")}
                                for k, v in rb["propulsion"].items()}}
    r["with_guard"] = {"mass_g": rg["mass_g"], "acceptance": rg["acceptance"], "cog_mm": rg["cog_mm"], "arm_crash_sf": rg["arm"]["crash_sf"],
                       "tightest_gaps": rg["tightest_gaps"], "failures": rg["failures"],
                       "warnings": rg.get("warnings", []),
                       "propulsion": {k: {kk: v[kk] for kk in ("thrust_to_weight_nominal", "hover_throttle", "hover_time_min")}
                                      for k, v in rg["propulsion"].items()}}
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "checks.json"), "w") as f:
        json.dump(r, f, indent=1)
    sys.exit(1 if r["failures"] or rg["failures"] or rb["failures"] else 0)


if __name__ == "__main__":
    main()

"""Fly the CAD-derived NF-3D in the neuroflight-fc simulator.

1. Build a vehicle parameter set from this repo's own analyses:
     mass, inertia, arm      <- out/mech_checks.json   (build123d mass model)
     thrust_max, curve, yaw  <- out/propulsion.json    (motor/prop/battery model)
   for two corners: `nominal` and `pessimistic` (ct -15 %, cp +20 %, mass +5 %).
   Everything else (drag, motor lag, latency, rates) keeps the neuroflight-fc
   nominal value; its randomization ranges are applied on top.
2. Run the neuroflight-fc evaluation suite (train/eval.py) on that vehicle with
   the firmware sensing path (`fw3`: filtered gyro, flow/ToF Kalman filter,
   firmware Mahony) for the cascaded PID fallback and the hover_ctbr_v3 policy.
3. Pass/fail against requirements declared below, before seeing the results.

    python -m flight.fly [--n 256]       (needs NEUROFLIGHT_FC or ../neuroflight-fc)
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NF = os.environ.get("NEUROFLIGHT_FC", os.path.join(os.path.dirname(ROOT), "neuroflight-fc"))
POLICY = os.path.join(NF, "train", "policies", "hover_ctbr_v3.nfpol.json")

FT_POLICY = os.path.join(ROOT, "runs", "nf3d_ft", "policy.nfpol.json")

# Acceptance = non-inferiority against the vehicle neuroflight-fc was developed
# on (its own config/vehicle_nominal.yaml), evaluated the same way in the same
# run: success on each gated scenario may be at most MARGIN below the reference.
# (A first version used absolute thresholds, 99 % for PID; the reference vehicle
# itself scores 97-99 % with the firmware sensing path, so that bar measured the
# estimator, not the airframe. Recorded here so the change is visible.)
GATED = ("hover_1m/randomized", "tilt_60/randomized", "throw/randomized")
MARGIN = 0.03
# controller on the NF-3D -> controller on the reference it must match
GATES = {"PID": "PID", "nf3d": "v3"}
SCENARIOS = ("hover_1m", "tilt_60", "throw", "upside_down")


def cad_vehicle(nf_nominal: dict, corner: str) -> dict:
    with open(os.path.join(ROOT, "out", "mech_checks.json")) as f:
        mech = json.load(f)
    with open(os.path.join(ROOT, "out", "propulsion.json")) as f:
        prop = json.load(f)[corner]
    v = copy.deepcopy(nf_nominal)
    v["mass"] = round(prop["mass_kg"], 4)  # propulsion json holds the corner's mass
    scale = prop["mass_kg"] / (mech["mass_g"] / 1000.0)
    I = mech["inertia_kgm2"]
    v["inertia"] = [round(I[i][i] * scale, 7) for i in range(3)]
    v["arm_xy"] = mech["arm_xy_m"]
    v["thrust_max"] = round(prop["thrust_max_n"], 4)
    v["thrust_curve_a"] = round(min(max(prop["thrust_curve_a"], 0.0), 1.0), 4)
    v["yaw_torque_coeff"] = round(prop["yaw_torque_coeff_m"], 5)
    return v


def ref_ref(ref, ctrl, grp):
    return ref[ctrl][grp]["success"]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=256)
    ap.add_argument("--threads", type=int, default=4)
    a = ap.parse_args(argv)
    sys.path.insert(0, NF)
    import train.eval as ev
    from sim.params import load_nominal

    nf_nom = load_nominal()
    pol = ev.TorchPolicy(POLICY)
    ctrls = {"v3": lambda n, nom: pol, "PID": ev.make_pid}
    if os.path.exists(FT_POLICY):
        ft = ev.TorchPolicy(FT_POLICY)
        ctrls["nf3d"] = lambda n, nom: ft
    else:
        print("note: no fine-tuned policy yet (python -m flight.retrain); gating PID only")
    report = {"failures": [], "vehicles": {}, "results": {}}
    md = ["# Flight validation (neuroflight-fc simulator, firmware sensing path `fw3`)", "",
          f"Controllers: `PID` = neuroflight-fc cascaded fallback, `v3` = hover_ctbr_v3 as trained on the reference "
          f"vehicle, `nf3d` = v3 fine-tuned on this airframe (flight/retrain.py). Gate: NF-3D success on "
          f"{', '.join(GATED)} at most {MARGIN*100:.0f} points below the reference vehicle "
          f"(PID vs PID, nf3d vs v3).", ""]
    for corner in ("reference", "nominal", "pessimistic"):
        veh = copy.deepcopy(nf_nom) if corner == "reference" else cad_vehicle(nf_nom, corner)
        report["vehicles"][corner] = veh
        with open(os.path.join(ROOT, "out", f"vehicle_{corner}.yaml"), "w") as f:
            yaml.safe_dump(veh, f, sort_keys=False)
        ev.load_nominal = lambda veh=veh: copy.deepcopy(veh)  # eval.build() reads the vehicle through this
        res = ev.evaluate(ctrls, n=a.n, n_ext=max(16, a.n // 8), seed=123, extremes=True,
                          scenarios=SCENARIOS, threads=a.threads, sensing="fw3")
        report["results"][corner] = res
        md += [f"## {corner} vehicle", "",
               "```", yaml.safe_dump({k: veh[k] for k in ("mass", "inertia", "arm_xy", "thrust_max",
                                                          "thrust_curve_a", "yaw_torque_coeff")},
                                     sort_keys=False).strip(), "```", "",
               ev.format_tables(res), ""]
        print(f"[{corner}] " + ", ".join(f"{c} {g.split('/')[0]} {res[c][g]['success']*100:.1f}%"
                                          for c in res for g in GATED), flush=True)
        if corner == "reference":
            continue
        ref = report["results"]["reference"]
        for ctrl, ref_ctrl in GATES.items():
            if ctrl not in res:
                continue
            for grp in GATED:
                got, need = res[ctrl][grp]["success"], ref[ref_ctrl][grp]["success"] - MARGIN
                if got < need:
                    report["failures"].append(f"{corner}: {ctrl} {grp} {got*100:.1f} % < reference "
                                              f"{ref_ref(ref, ref_ctrl, grp)*100:.1f} % - {MARGIN*100:.0f}")
    md += ["## Result", "", "PASS" if not report["failures"] else "FAIL"] + [f"- {x}" for x in report["failures"]]
    with open(os.path.join(ROOT, "out", "flight.json"), "w") as f:
        json.dump(report, f, indent=1)
    with open(os.path.join(ROOT, "out", "FLIGHT-REPORT.md"), "w") as f:
        f.write("\n".join(md) + "\n")
    print("FAIL" if report["failures"] else "PASS", *report["failures"], sep="\n  ")
    sys.exit(1 if report["failures"] else 0)


if __name__ == "__main__":
    main()

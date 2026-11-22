"""Stress tests for the NF-3D Lite, aimed at the weak points in the design review.

    python -m lite.stress [--n 128] [--corner nominal|pessimistic]  -> lite/out/stress_<corner>.json

Each case applies a fault to every simulated drone, on top of the chosen motor corner
(--corner nominal|pessimistic) and neuroflight-fc's randomization, then runs the same
angle-mode recovery scenarios as lite/fly.py (tilt_60, throw; upside_down reported).
It also reports the hover throttle the fault leaves: above 0.85 there is too little
thrust left over to steer, whatever the attitude result says.

Cases (review finding in brackets):
  one_motor_85      one motor 15 % weak, e.g. a worn brush or a heat-softened tilted mount  [2]
  one_motor_tilt    ~5 deg tilted motor, modelled as 12 % thrust loss on it + yaw imbalance [2]
  vibration_3x      gyro / accel noise at 3x neuroflight-fc's worst case (hard-mounted IMU)  [8]
  slow_motors       motor lag x2 and 8 ms latency                                            [-]
  heavy_15          15 % heavier (guards, PETG, a bigger battery)                           [3, 4]
  empty_battery     thrust at 3.4 V/cell (end of flight)                                     [-]
  combined          one motor 90 %, lag x1.5, vibration x2, 6 ms latency, 3.6 V battery      [-]
Acceptance (declared before the run): gated recovery >= 90 % for every case except
empty_battery and combined, which are reported. Hover throttle <= 0.85 for every gated case.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import torch

from . import fly

HERE = os.path.dirname(os.path.abspath(__file__))


def cases(checks):
    p = checks["propulsion"]["pessimistic"]
    empty = p["thrust_to_weight_empty"] / p["thrust_to_weight_nominal"]  # thrust ratio 3.4 V vs 3.7 V
    mid = 1 - (1 - empty) * 0.5                                          # ~3.6 V/cell

    def one_motor(k):
        def f(q):
            q.params.thrust_max[:, 0] *= k
        return f

    def tilt(q):
        q.params.thrust_max[:, 0] *= 0.88
        q.params.yaw_k[:] *= 1.0       # yaw imbalance comes from the unequal thrusts themselves

    def vib(k):
        def f(q):
            q.params.gyro_noise_std[:] = 0.05 * k
            q.params.accel_noise_std[:] = 0.3 * k
        return f

    def slow(q):
        q.params.tau_up *= 2.0
        q.params.tau_down *= 2.0
        q.params.latency_steps[:] = min(int(round(0.008 / q.dt)), q.max_delay)

    def heavy(q):
        q.params.mass *= 1.15

    def batt(k):
        def f(q):
            q.params.thrust_max *= k
        return f

    def combined(q):
        q.params.thrust_max[:, 0] *= 0.90
        q.params.thrust_max *= mid
        q.params.tau_up *= 1.5
        q.params.tau_down *= 1.5
        q.params.gyro_noise_std[:] = 0.10
        q.params.accel_noise_std[:] = 0.6
        q.params.latency_steps[:] = min(int(round(0.006 / q.dt)), q.max_delay)

    # (name, mutate, gated, thrust factor on the weakest motor set, mass factor)
    return [("one_motor_85", one_motor(0.85), True, (0.85 + 3) / 4, 1.0),
            ("one_motor_tilt", tilt, True, (0.88 + 3) / 4, 1.0),
            ("vibration_3x", vib(3.0), True, 1.0, 1.0),
            ("slow_motors", slow, True, 1.0, 1.0),
            ("heavy_15", heavy, True, 1.0, 1.15),
            ("empty_battery", batt(empty), False, empty, 1.0),
            ("combined", combined, False, mid * (0.90 + 3) / 4, 1.0)]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=128)
    ap.add_argument("--corner", choices=("nominal", "pessimistic"), default="nominal")
    a = ap.parse_args(argv)
    sys.path.insert(0, fly.NF)
    import train.eval as ev
    from sim.params import load_nominal
    from lite.frame import load_spec
    spec = load_spec()
    with open(os.path.join(fly.OUT, "checks.json")) as f:
        checks = json.load(f)
    veh = fly.lite_vehicle(load_nominal(), spec, checks, a.corner)
    base_u = checks["propulsion"][a.corner]["hover_throttle"]   # propulsion model, nominal voltage
    out = {"corner": a.corner, "cases": {}, "failures": []}
    for name, mut, gated, thrust_k, mass_k in cases(checks):
        r = fly.run_vehicle(ev, veh, spec, a.n, 16, 77, mutate=mut, extremes=False)
        hover_u = base_u * mass_k / thrust_k        # thrust ~linear in throttle (A ~ 0.03)
        g = {sc: min(r[f"{sc}/nominal"]["success"], r[f"{sc}/randomized"]["success"]) for sc in fly.GATED + fly.REPORTED}
        sat = max(v["saturated_frac"] for v in r.values())
        out["cases"][name] = {"gated": gated, "worst_success": g, "hover_throttle": round(hover_u, 3),
                              "max_saturated_frac": round(sat, 3)}
        worst = min(g[sc] for sc in fly.GATED)
        print(f"{name:15s} tilt_60 {g['tilt_60']*100:5.1f}%  throw {g['throw']*100:5.1f}%  "
              f"upside_down {g['upside_down']*100:5.1f}%  hover throttle {hover_u:.2f}  saturated {sat*100:.0f}%"
              + ("" if gated else "  (reported)"), flush=True)
        if gated and worst < 0.90:
            out["failures"].append(f"{name}: recovery {worst*100:.1f} % < 90 %")
        if gated and hover_u > 0.85:
            out["failures"].append(f"{name}: hover throttle {hover_u:.2f} > 0.85")
    with open(os.path.join(fly.OUT, f"stress_{a.corner}.json"), "w") as f:
        json.dump(out, f, indent=1)
    print("FAIL" if out["failures"] else "PASS", *out["failures"], sep="\n  ")
    return 1 if out["failures"] else 0


if __name__ == "__main__":
    sys.exit(main())

"""Steady-state propulsion model: battery -> ESC -> motor -> propeller.

Model (per motor, all four identical):
  motor   V_m = u * V_bat,  V_m = I Rm + omega / Kv_rad,  Kt = 1 / Kv_rad,
          I = Q / Kt + I0(V_m)            (I0 scaled linearly with voltage)
  prop    T = ct rho n^2 D^4 * duct_gain,  Q = cp rho n^2 D^5 / (2 pi)
  battery V_bat = N_cells * V_cell - I_total * R_pack
Solved by fixed-point iteration on V_bat and a 1-D root find on omega.

Outputs feed the flight simulator (thrust_max, thrust_curve_a, motor tau) and
the requirements check (T/W, hover throttle, hover time, ESC current).
`python -m prop.propulsion` prints the report and writes out/propulsion.json.
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
from scipy.optimize import brentq

from mech.frame import load_spec

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Propulsion:
    def __init__(self, spec: dict, ct_scale: float = 1.0, cp_scale: float = 1.0):
        self.s = spec
        m, p, e = spec["motor"], spec["propeller"], spec["environment"]
        self.kv_rad = m["kv_rpm_per_v"] * 2 * math.pi / 60.0   # rad/s per V
        self.kt = 1.0 / self.kv_rad                            # N m / A
        self.rm, self.i0, self.i0_v = m["rm_ohm"], m["i0_a"], m["i0_ref_v"]
        self.D = spec["geometry_mm"]["prop_diameter"] / 1000.0
        self.rho = e["rho"]
        self.ct, self.cp, self.gain = p["ct"] * ct_scale, p["cp"] * cp_scale, p["duct_thrust_gain"]
        self.J = m["rotor_inertia_kgm2"] + p["inertia_kgm2"]
        el = spec["electrical"]
        self.cells, self.r_pack = el["cells"], el["pack_ir_ohm"]
        self.esc_eff = el["esc_efficiency"]

    # ---- prop
    def thrust(self, omega):
        n = omega / (2 * math.pi)
        return self.ct * self.rho * n * n * self.D ** 4 * self.gain

    def torque(self, omega):
        n = omega / (2 * math.pi)
        return self.cp * self.rho * n * n * self.D ** 5 / (2 * math.pi)

    # ---- motor at a given terminal voltage
    def motor_point(self, v_m):
        if v_m <= 0:
            return 0.0, 0.0
        i0 = self.i0 * v_m / self.i0_v

        def f(w):
            i = self.torque(w) / self.kt + i0
            return v_m - i * self.rm - w / self.kv_rad

        w = brentq(f, 0.0, v_m * self.kv_rad)
        return w, self.torque(w) / self.kt + i0

    def operating_point(self, u, v_cell, avionics_w=0.0):
        """Throttle u in [0,1] on all 4 motors. Returns dict per vehicle."""
        v_bat = self.cells * v_cell
        for _ in range(60):
            w, i_m = self.motor_point(u * v_bat)
            i_bat = 4 * i_m * u / self.esc_eff + avionics_w / max(v_bat, 1e-3)  # ESC: battery current = duty * motor current
            v_new = self.cells * v_cell - i_bat * self.r_pack
            if abs(v_new - v_bat) < 1e-6:
                break
            v_bat = 0.5 * v_bat + 0.5 * v_new
        return {"u": u, "omega": w, "rpm": w * 60 / (2 * math.pi), "thrust_per_motor_n": self.thrust(w),
                "motor_current_a": i_m, "battery_current_a": i_bat, "v_bat": v_bat,
                "elec_power_w": v_bat * i_bat}

    def throttle_for_thrust(self, thrust_total, v_cell, avionics_w=0.0):
        f = lambda u: 4 * self.operating_point(u, v_cell, avionics_w)["thrust_per_motor_n"] - thrust_total
        if f(1.0) < 0:
            return None
        return brentq(f, 1e-3, 1.0)

    def motor_tau(self, omega):
        """Small-signal mechanical time constant of motor+prop at speed omega."""
        dq = 2 * self.torque(omega) / max(omega, 1e-6)   # dQ/domega for Q ~ omega^2
        damping = self.kt / (self.rm * self.kv_rad) + dq
        return self.J / damping


def analyse(spec: dict | None = None, mass_kg: float | None = None, corner: str = "nominal") -> dict:
    spec = spec or load_spec()
    pe = spec["pessimistic"] if corner == "pessimistic" else {"ct_scale": 1.0, "cp_scale": 1.0, "mass_scale": 1.0}
    pr = Propulsion(spec, pe["ct_scale"], pe["cp_scale"])
    el, req = spec["electrical"], spec["requirements"]
    g = spec["environment"]["g"]
    if mass_kg is None:
        path = os.path.join(ROOT, "out", "mech_checks.json")
        with open(path) as f:
            mass_kg = json.load(f)["mass_g"] / 1000.0
    mass_kg *= pe["mass_scale"]
    W = mass_kg * g
    av = el["avionics_w"]
    out = {"corner": corner, "mass_kg": mass_kg, "weight_n": W, "failures": []}

    # full-throttle thrust at full, nominal and empty cell voltage
    for tag, vc in (("full", el["cell_v_full"]), ("nominal", el["cell_v_nominal"]), ("empty", el["cell_v_empty"])):
        op = pr.operating_point(1.0, vc, av)
        out[f"max_{tag}"] = op
    tmax_nom = out["max_nominal"]["thrust_per_motor_n"]
    out["thrust_to_weight_nominal"] = 4 * tmax_nom / W
    out["thrust_to_weight_empty"] = 4 * out["max_empty"]["thrust_per_motor_n"] / W

    # hover
    u_h = pr.throttle_for_thrust(W, el["cell_v_nominal"], av)
    hov = pr.operating_point(u_h, el["cell_v_nominal"], av)
    out["hover"] = hov
    usable_ah = el["capacity_mah"] / 1000.0 * el["usable_fraction"]
    out["hover_time_min"] = usable_ah / hov["battery_current_a"] * 60.0
    out["hover_rotor_hz"] = hov["omega"] / (2 * math.pi)
    out["hover_blade_pass_hz"] = out["hover_rotor_hz"] * 3
    out["motor_tau_hover_s"] = pr.motor_tau(hov["omega"])

    # thrust curve fit T(w)/Tmax = A w^2 + (1-A) w with w = throttle (INTERFACES.md §2)
    us = np.linspace(0.05, 1.0, 40)
    ts = np.array([pr.operating_point(u, el["cell_v_nominal"], av)["thrust_per_motor_n"] for u in us]) / tmax_nom
    X = us ** 2 - us
    A = float(np.dot(X, ts - us) / np.dot(X, X))
    out["thrust_curve_a"] = A
    out["thrust_curve_fit_rms"] = float(np.sqrt(np.mean((A * us ** 2 + (1 - A) * us - ts) ** 2)))
    out["thrust_max_n"] = tmax_nom
    out["yaw_torque_coeff_m"] = pr.torque(out["max_nominal"]["omega"]) / tmax_nom

    # requirements
    if out["thrust_to_weight_nominal"] < req["min_thrust_to_weight"]:
        out["failures"].append(f"T/W {out['thrust_to_weight_nominal']:.2f} < {req['min_thrust_to_weight']}")
    if u_h > req["max_hover_throttle"]:
        out["failures"].append(f"hover throttle {u_h:.2f} > {req['max_hover_throttle']}")
    if out["hover_time_min"] < req["min_hover_time_min"]:
        out["failures"].append(f"hover time {out['hover_time_min']:.1f} min < {req['min_hover_time_min']}")
    cap_ah = el["capacity_mah"] / 1000.0
    out["battery_c_hover"] = hov["battery_current_a"] / cap_ah
    out["battery_c_wot"] = out["max_full"]["battery_current_a"] / cap_ah
    if out["battery_c_wot"] > el["c_rating_burst"]:
        out["failures"].append(f"WOT draw {out['battery_c_wot']:.0f} C > burst rating {el['c_rating_burst']} C")
    if out["battery_c_hover"] > el["c_rating_continuous"]:
        out["failures"].append(f"hover draw {out['battery_c_hover']:.0f} C > continuous rating")
    i_esc = out["max_full"]["motor_current_a"]
    out["esc_current_fraction"] = i_esc / el["esc_continuous_a"]
    if out["esc_current_fraction"] > req["max_esc_current_fraction"]:
        out["failures"].append(f"ESC per-channel current {i_esc:.1f} A > {req['max_esc_current_fraction']} x rating")
    if i_esc > spec["motor"]["max_current_a"]:
        out["failures"].append(f"motor current {i_esc:.1f} A > motor max {spec['motor']['max_current_a']} A")
    return out


def report(r):
    print(f"--- {r['corner']} corner ---")
    print(f"AUW {r['mass_kg']*1000:.1f} g, weight {r['weight_n']:.2f} N")
    for tag in ("full", "nominal", "empty"):
        o = r[f"max_{tag}"]
        print(f"  WOT @{tag:7s}: {o['thrust_per_motor_n']:.2f} N/motor, {o['rpm']:.0f} rpm, "
              f"{o['motor_current_a']:.1f} A/motor, battery {o['battery_current_a']:.1f} A at {o['v_bat']:.1f} V")
    h = r["hover"]
    print(f"  T/W nominal {r['thrust_to_weight_nominal']:.2f} (empty {r['thrust_to_weight_empty']:.2f})")
    print(f"  hover: throttle {h['u']:.3f}, {h['rpm']:.0f} rpm ({r['hover_rotor_hz']:.0f} Hz rotor, "
          f"{r['hover_blade_pass_hz']:.0f} Hz blade pass), {h['battery_current_a']:.2f} A, {h['elec_power_w']:.1f} W")
    print(f"  battery: hover {r['battery_c_hover']:.1f} C, full throttle {r['battery_c_wot']:.0f} C; "
          f"ESC channel at {r['esc_current_fraction']*100:.0f} % of rating")
    print(f"  hover time {r['hover_time_min']:.1f} min, motor tau at hover {r['motor_tau_hover_s']*1000:.1f} ms")
    print(f"  sim params: thrust_max {r['thrust_max_n']:.2f} N, thrust_curve_a {r['thrust_curve_a']:.3f} "
          f"(fit rms {r['thrust_curve_fit_rms']:.4f}), yaw coeff {r['yaw_torque_coeff_m']:.4f} m")
    print("FAIL" if r["failures"] else "PASS", *r["failures"], sep="\n  ")


def main():
    res = {c: analyse(corner=c) for c in ("nominal", "pessimistic")}
    for r in res.values():
        report(r)
    with open(os.path.join(ROOT, "out", "propulsion.json"), "w") as f:
        json.dump(res, f, indent=1)
    sys.exit(1 if any(r["failures"] for r in res.values()) else 0)


if __name__ == "__main__":
    main()

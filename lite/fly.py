"""Fly the NF-3D Lite in the neuroflight-fc simulator, the way it will really be flown.

The Lite has an IMU and nothing else: no optical flow, ToF or barometer. So it is an
angle-mode drone. The Xbox stick commands a tilt, the firmware holds that tilt, and
the pilot works the throttle. Position hold and the hover_ctbr_v3 policy do not
apply (both need position sensing). This script tests the part the firmware owns:

  * sensing: filtered gyro + the firmware Mahony on the accelerometer (no KF reference)
  * inner loop: the neuroflight-fc rate PID (gains scale with inertia)
  * outer loop: angle mode, P on the attitude error -> body-rate setpoint, yaw rate 0
  * collective: the pilot's hover throttle for the *nominal* mass, tilt-compensated.
    Altitude is the pilot's job, so the floor is removed and altitude is not scored.

Vehicle: mass, inertia, arm, thrust curve, yaw coefficient from lite/out/checks.json
(nominal and pessimistic propulsion corners); motor lag, rate limits from spec `sim`.
neuroflight-fc's own randomization ranges are applied on top, plus its extremes.

Scenarios (from neuroflight-fc train/eval.py initial states): tilt_60 and throw must
recover to level (tilt < 10 deg, |rate| < 1 rad/s, held for the last second of 3 s);
upside_down is reported only (angle-mode flip recovery at T/W 1.6 is not a requirement).
A 20-degree roll stick step gives rise time and overshoot.

Acceptance, declared before the run: success >= 95 % on tilt_60 and throw for the
nominal corner (nominal + randomized), >= 90 % for pessimistic; step overshoot < 25 %.

    python -m lite.fly [--n 256]     (needs NEUROFLIGHT_FC or ../neuroflight-fc)
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
NF = os.environ.get("NEUROFLIGHT_FC", os.path.join(os.path.dirname(ROOT), "neuroflight-fc"))
OUT = os.path.join(HERE, "out")

HORIZON_S = 3.0
HOLD_S = 1.0
TILT_TOL = math.radians(10.0)
RATE_TOL = 1.0
GATED = ("tilt_60", "throw")
REPORTED = ("upside_down",)
NEED = {"nominal": 0.95, "pessimistic": 0.90}
MAX_OVERSHOOT = 0.25


def lite_vehicle(nf_nominal: dict, spec: dict, checks: dict, corner: str) -> dict:
    p = checks["propulsion"][corner]
    s = spec["sim"]
    v = copy.deepcopy(nf_nominal)
    scale = p["mass_kg"] / (checks["mass_g"] / 1000.0)
    I = checks["inertia_kgm2"]
    v["mass"] = round(p["mass_kg"], 4)
    v["inertia"] = [round(I[i][i] * scale, 8) for i in range(3)]
    v["arm_xy"] = checks["arm_xy_m"]
    v["thrust_max"] = round(p["thrust_max_n"], 4)
    # the simulator inverts T = Tmax (A w^2 + (1-A) w) by dividing by A, so A = 0 (the
    # pessimistic fit: linear) is held at 0.02, a negligible change to the curve
    v["thrust_curve_a"] = round(min(max(p["thrust_curve_a"], 0.02), 1.0), 4)
    v["yaw_torque_coeff"] = round(p["yaw_torque_coeff_m"], 5)
    v["motor_tau_up"], v["motor_tau_down"] = s["motor_tau_up"], s["motor_tau_down"]
    v["idle_command"] = s["idle_command"]
    v["max_body_rate"] = list(s["max_body_rate"])
    if s.get("lin_drag_scale_by_mass"):
        k = v["mass"] / nf_nominal["mass"]
        v["lin_drag"] = [round(d * k, 4) for d in nf_nominal["lin_drag"]]
        v["rot_drag"] = [d * k for d in nf_nominal["rot_drag"]]
    return v


class AngleMode:
    """Outer loop of an angle-mode flight controller on the firmware attitude estimate."""

    def __init__(self, env, veh: dict, kp, tilt_sp=None):
        self.env, self.veh = env, veh
        self.kp = torch.tensor([kp[0], kp[1]], dtype=torch.float32)
        self.rmax = torch.tensor(veh["max_body_rate"], dtype=torch.float32)
        self.hover_f = veh["mass"] * veh["gravity"]       # pilot trims for the nominal mass
        self.fmax = 4 * veh["thrust_max"]
        self.tilt_sp = tilt_sp                            # (N,3) desired body-z in world, or None = level

    def __call__(self):
        R = self.env._R_att()                             # firmware estimate, body->world
        b3 = R[:, :, 2]
        n = b3.shape[0]
        d3 = self.tilt_sp if self.tilt_sp is not None else torch.tensor([0.0, 0.0, 1.0]).expand(n, 3)
        c = torch.cross(b3, d3, dim=-1)                   # world-frame rotation axis * sin
        s = c.norm(dim=-1)
        ang = torch.atan2(s, (b3 * d3).sum(-1))
        axis = torch.where(s.unsqueeze(-1) > 1e-6, c / s.clamp_min(1e-6).unsqueeze(-1),
                           R[:, :, 0])                    # exactly inverted: roll about body x
        e_b = (R.transpose(1, 2) @ (axis * ang.unsqueeze(-1)).unsqueeze(-1)).squeeze(-1)
        rate = torch.zeros(n, 3)
        rate[:, :2] = self.kp * e_b[:, :2]
        rate = torch.maximum(torch.minimum(rate, self.rmax), -self.rmax)
        cos_t = b3[:, 2].clamp(0.5, 1.0)
        F = torch.where(b3[:, 2] > 0, self.hover_f / cos_t, torch.full((n,), 0.3 * self.hover_f))
        F = F.clamp(0.0, self.fmax)
        a0 = F / self.fmax * 2 - 1
        return torch.cat((a0.unsqueeze(-1), rate / self.rmax), -1).clamp(-1, 1)


def run_vehicle(ev, veh, spec, n, n_ext, seed):
    ev.load_nominal = lambda: copy.deepcopy(veh)
    ev.SENSING["imu"] = dict(gyro_filter=True, estimator=False, mahony=True, mahony_kf_ref=False)
    scen = GATED + REPORTED
    env, groups, _ = ev.build(n, n_ext, seed, True, scen, "imu")
    env.cfg.oob_dist = 1e9                                # position is the pilot's job
    env.quad.floor_z = -1e9                               # altitude too
    ctrl = AngleMode(env, veh, spec["sim"]["angle_kp"])
    steps = int(HORIZON_S * veh["control_rate_hz"])
    hold = int(HOLD_S * veh["control_rate_hz"])
    rows = env.quad.n
    ok = torch.ones(rows, dtype=torch.bool)
    dead = torch.zeros(rows, dtype=torch.bool)
    settle = torch.full((rows,), float("nan"))
    last_bad = torch.zeros(rows)
    sat = torch.zeros(rows)
    for k in range(steps):
        _, _, done, _ = env.step(ctrl())
        dead |= done
        q = env.quad
        tilt = torch.acos(q.R[:, 2, 2].clamp(-1, 1))
        bad = (tilt > TILT_TOL) | (q.omega.abs().amax(-1) > RATE_TOL) | ~torch.isfinite(tilt)
        last_bad = torch.where(bad, torch.full_like(last_bad, (k + 1) / veh["control_rate_hz"]), last_bad)
        if k >= steps - hold:
            ok &= ~bad
        sat += (q.motor > 0.99).any(-1).float()
    ok &= ~dead
    settle = torch.where(ok, last_bad, settle)
    res = {}
    for g in groups:
        sl = g.sl
        s = settle[sl][ok[sl]]
        res[g.name] = {"success": float(ok[sl].float().mean()),
                       "settle_s_median": float(s.median()) if s.numel() else None,
                       "settle_s_p95": float(s.quantile(0.95)) if s.numel() > 1 else None,
                       "saturated_frac": float((sat[sl] / steps).mean())}
    return res


def step_response(ev, veh, spec, deg=20.0):
    """Nominal vehicle, level hover, 20-degree roll stick step for 1 s."""
    ev.load_nominal = lambda: copy.deepcopy(veh)
    env, groups, _ = ev.build(1, 1, 7, False, ("hover_1m",), "imu")
    env.cfg.oob_dist, env.quad.floor_z = 1e9, -1e9
    q = env.quad
    idx = torch.arange(q.n)
    w = q.hover_motor_state(idx)
    q.set_state(idx, vel=torch.zeros(q.n, 3), omega=torch.zeros(q.n, 3),
                quat=torch.tensor([[1.0, 0, 0, 0]]).expand(q.n, 4).clone(), motor=w, cmd=w)
    env.reinit_sensing(idx)
    t = math.radians(deg)
    d3 = torch.tensor([[0.0, -math.sin(t), math.cos(t)]]).expand(q.n, 3)   # roll right
    ctrl = AngleMode(env, veh, spec["sim"]["angle_kp"])
    rate = veh["control_rate_hz"]
    for _ in range(int(0.3 * rate)):
        env.step(ctrl())
    ctrl.tilt_sp = d3
    roll, est = [], []
    rl = lambda R: math.degrees(math.atan2(float(R[2, 1]), float(R[2, 2])))
    for _ in range(int(1.0 * rate)):
        env.step(ctrl())
        roll.append(rl(q.R[0]))
        est.append(rl(env._R_att()[0]))
    # The loop is judged on the attitude it controls (the firmware estimate). The true
    # tilt then creeps past it: with no flow/baro the Mahony filter's only reference is
    # the accelerometer, which reads "level" while the drone accelerates sideways. That
    # drift is reported, not gated; the pilot corrects it with the stick, as on any
    # IMU-only angle-mode micro.
    peak = max(est, key=abs)
    t90 = next((i / rate for i, r in enumerate(est) if abs(r) >= 0.9 * deg), None)
    return {"step_deg": deg, "t90_s": t90, "peak_deg": round(abs(peak), 2),
            "overshoot": round(abs(peak) / deg - 1, 3), "final_deg": round(abs(est[-1]), 2),
            "true_tilt_after_1s_deg": round(abs(roll[-1]), 2)}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=256)
    a = ap.parse_args(argv)
    sys.path.insert(0, NF)
    sys.path.insert(0, ROOT)
    import train.eval as ev
    from sim.params import load_nominal
    from lite.frame import load_spec
    spec = load_spec()
    with open(os.path.join(OUT, "checks.json")) as f:
        checks = json.load(f)
    nf_nom = load_nominal()
    out = {"vehicles": {}, "results": {}, "step": {}, "failures": []}
    for corner in ("nominal", "pessimistic"):
        veh = lite_vehicle(nf_nom, spec, checks, corner)
        out["vehicles"][corner] = veh
        res = run_vehicle(ev, veh, spec, a.n, max(16, a.n // 8), 123)
        out["results"][corner] = res
        out["step"][corner] = step_response(ev, veh, spec)
        for sc in GATED:
            for mode in ("nominal", "randomized"):
                s = res[f"{sc}/{mode}"]["success"]
                if s < NEED[corner]:
                    out["failures"].append(f"{corner} {sc}/{mode}: {s*100:.1f} % < {NEED[corner]*100:.0f} %")
        if out["step"][corner]["overshoot"] > MAX_OVERSHOOT:
            out["failures"].append(f"{corner} step overshoot {out['step'][corner]['overshoot']*100:.0f} %")
        print(f"[{corner}] " + ", ".join(f"{g} {r['success']*100:.1f}% (settle {r['settle_s_median'] or float('nan'):.2f}s)"
                                         for g, r in res.items() if "/" in g and not g.split("/")[0].count("=")), flush=True)
        worst = sorted((r["success"], g) for g, r in res.items() if "=" in g)[:4]
        print(f"   worst extremes: {[(g, round(s*100, 1)) for s, g in worst]}")
        print(f"   step: {out['step'][corner]}")
    # How slow can the motors be before the stock rate gains stop working? (reported:
    # the 40/60 ms lag in spec `sim` is an estimate until the bench test measures it)
    out["tau_margin"] = {}
    for ts in (1.5, 2.0, 3.0):
        veh = lite_vehicle(nf_nom, spec, checks, "pessimistic")
        veh["motor_tau_up"] *= ts
        veh["motor_tau_down"] *= ts
        r = run_vehicle(ev, veh, spec, max(32, a.n // 4), 16, 321)
        out["tau_margin"][f"x{ts}"] = round(min(v["success"] for g, v in r.items() if "=" not in g and g.split("/")[0] in GATED), 3)
    print("   pessimistic, motor lag scaled (worst gated success):", out["tau_margin"])
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "flight.json"), "w") as f:
        json.dump(out, f, indent=1)
    print("FAIL" if out["failures"] else "PASS", *out["failures"], sep="\n  ")
    return 1 if out["failures"] else 0


if __name__ == "__main__":
    sys.exit(main())

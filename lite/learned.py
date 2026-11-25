"""Could the Lite fly neuroflight-fc's learned controller, and where should it run?

    python -u -m lite.learned [--n 128] [--seed 11]      -> lite/out/learned.json

The learned controller is neuroflight-fc's hover_ctbr_v3 policy: a 19-64-64-4 MLP that maps the
estimated state (attitude, rates, position/velocity error from the flow/ToF filter, last action)
to collective thrust + body rates (CTBR), at 500 Hz, ahead of the 1 kHz rate PID. It was trained
for neuroflight's own 3" vehicle, so on the Lite it runs zero-shot here.

Where it runs is modelled as the delay between the IMU sample and the motor command:
  onboard   3 ms  (the policy on the flight board's ESP32, like the classical loop today)
  wifi_10   10 ms (laptop over Wi-Fi/UDP, best case)
  wifi_25   25 ms (typical round trip on a busy 2.4 GHz link)
  wifi_50   50 ms
  wifi_100  100 ms (a Wi-Fi hiccup; real drop-outs are longer and are not modelled)
Randomized rows add 0-8 ms of extra delay on top (as in neuroflight's training randomization).

Each delay flies neuroflight-fc's eval scenarios (hover_1m, tilt_60, throw; fw3 sensing = the
Lite's flow/ToF + Mahony path) on identical vehicles and initial states, with:
  RL@Lite, RL@NF  as above (hover_ctbr_v3)
  PID@Lite        neuroflight's cascaded PID baseline on the Lite
  Lite            the Lite's own onboard position hold (lite/autonomy.py Autopilot, the firmware's twin)
"""
from __future__ import annotations

import argparse
import copy
import json
import os

import torch

from .autonomy import HORIZON_A, MIN_TW, OUT, Autopilot, G, _eval_mod, _vehicles

NF = os.environ.get("NEUROFLIGHT_FC", os.path.join(os.path.dirname(__file__), "..", "..", "neuroflight-fc"))
POLICY = os.path.join(NF, "train", "policies", "hover_ctbr_v3.nfpol.json")
DELAYS = {"onboard": 0.003, "wifi_10": 0.010, "wifi_25": 0.025, "wifi_50": 0.050, "wifi_100": 0.100}
SCEN = ("hover_1m", "tilt_60", "throw")


def run(ev, veh, spec, delay, ctrl_name, n, seed, horizon_s, nf_ref=None):
    v = copy.deepcopy(veh)
    v["latency_s"] = delay
    rand0 = ev.load_randomization
    rnd = copy.deepcopy(rand0())
    rnd["latency_s"] = [delay, delay + 0.008]
    ev.load_nominal = lambda: copy.deepcopy(v)
    ev.load_randomization = lambda *a, **k: copy.deepcopy(rnd)
    try:
        env, groups, _ = ev.build(n, 1, seed, False, SCEN, "fw3")
    finally:
        ev.load_randomization = rand0
    if ctrl_name == "RL@Lite-cal":
        from flight.fly import CollectiveScaled
        ctrl = CollectiveScaled(ev.TorchPolicy(POLICY), v, nf_ref)
    elif ctrl_name.startswith("RL"):
        ctrl = ev.TorchPolicy(POLICY)
    elif ctrl_name.startswith("PID"):
        ctrl = ev.make_pid(env.n, v)
    else:
        ctrl = Autopilot(env, v, spec)
        ctrl.law.p_sp[:] = torch.tensor(ev.TARGET)
    res = ev.rollout(env, ctrl, horizon_s=horizon_s)
    out = ev.summarize(res, groups)
    qp = env.quad.params
    fly = (qp.thrust_max.sum(-1) / (qp.mass * G)) >= MIN_TW
    rows = {}
    for gr in groups:
        f = fly[gr.sl]
        m = out[gr.name]
        rows[gr.name] = {
            "success": float(res["success"][gr.sl][f].float().mean()) if f.any() else float("nan"),
            "err_med_m": m.get("err_med"),
            "unflyable": int((~f).sum()),
        }
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=128)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--all", action="store_true", help="also fly RL@Lite at every Wi-Fi delay")
    ap.add_argument("--corner", default="nominal", choices=("nominal", "pessimistic"))
    a = ap.parse_args(argv)
    torch.set_num_threads(max(1, os.cpu_count() or 1))
    from .frame import load_spec
    spec = load_spec()
    with open(os.path.join(OUT, "checks.json")) as f:
        checks = json.load(f)
    ev = _eval_mod()
    nom_load = ev.load_nominal
    nf_veh = nom_load()
    veh = _vehicles(spec, checks, ev)[a.corner]
    out = {"policy": os.path.basename(POLICY), "corner": a.corner, "n": a.n, "results": {}}
    for dname, d in DELAYS.items():
        for c in ("RL@Lite", "RL@Lite-cal", "RL@NF", "PID@Lite", "Lite"):
            if c.startswith("RL@Lite") and d > 0.003 and not a.all:
                continue                      # the transfer question only needs the onboard delay
            rows = run(ev, nf_veh if c == "RL@NF" else veh, spec, d, c, a.n, a.seed, HORIZON_A[a.corner], nf_veh)
            ev.load_nominal = nom_load
            out["results"].setdefault(dname, {})[c] = rows
            print(f"{dname:9s} {c:12s} " + "  ".join(f"{k} {v['success'] * 100:5.1f}%" for k, v in rows.items()), flush=True)
    with open(os.path.join(OUT, "learned.json"), "w") as f:
        json.dump(out, f, indent=1)
    print("wrote lite/out/learned.json")


if __name__ == "__main__":
    main()

"""Fine-tune the neuroflight-fc hover_ctbr_v3 policy for the CAD-derived NF-3D.

The v3 policy was trained on neuroflight-fc's pre-CAD vehicle (4.5 N motors,
0.046 m arms). NF-3D has weaker motors (3.34 N nominal, 2.56 N pessimistic),
longer arms and ~60 % more roll/pitch inertia, so it falls outside v3's
training distribution. This script fine-tunes v3 with the neuroflight-fc PPO
trainer and the v3 reward/smoothness config, on:
  nominal vehicle  = out/vehicle_nominal.yaml (from flight.fly)
  randomization    = neuroflight-fc config/randomization.yaml, with mass and
                     thrust ranges widened so the pessimistic corner is inside.

There is no v3 training checkpoint (only the exported nfpol), so the actor and
observation normalizer are loaded from the nfpol, a fresh critic is warmed up
with the actor frozen, then both train. Resumable: rerun after a restart.

    python -m flight.retrain [--iters 400] [--warmup 40]
Writes runs/nf3d_ft/{last.pt,policy.nfpol.json} under this repo.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys

import torch
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NF = os.environ.get("NEUROFLIGHT_FC", os.path.join(os.path.dirname(ROOT), "neuroflight-fc"))
V3 = os.path.join(NF, "train", "policies", "hover_ctbr_v3.nfpol.json")
OUT = os.path.join(ROOT, "runs", "nf3d_ft")


def patch_vehicle():
    """Make every neuroflight-fc module that loads the nominal vehicle see NF-3D."""
    sys.path.insert(0, NF)
    with open(os.path.join(ROOT, "out", "vehicle_nominal.yaml")) as f:
        veh = yaml.safe_load(f)
    import sim.params, sim.env, sim.quadrotor, train.ppo, train.priv_env  # noqa: E401,F401
    for name, mod in list(sys.modules.items()):
        if (name.startswith("sim.") or name.startswith("train.")) and hasattr(mod, "load_nominal"):
            mod.load_nominal = lambda *a, **k: copy.deepcopy(veh)
    with open(os.path.join(NF, "config", "randomization.yaml")) as f:
        rand = yaml.safe_load(f)
    with open(os.path.join(ROOT, "out", "vehicle_pessimistic.yaml")) as f:
        pess = yaml.safe_load(f)
    lo_thr = round(pess["thrust_max"] / veh["thrust_max"] - 0.05, 2)
    rand["thrust_max"] = [min(rand["thrust_max"][0], lo_thr), rand["thrust_max"][1]]
    rand["mass"] = [rand["mass"][0], max(rand["mass"][1], round(pess["mass"] / veh["mass"] + 0.05, 2))]
    os.makedirs(OUT, exist_ok=True)
    rpath = os.path.join(OUT, "randomization.yaml")
    with open(rpath, "w") as f:
        yaml.safe_dump(rand, f, sort_keys=False)
    return veh, rand, rpath


def nfpol_to_state(pol: dict, algo):
    """Load an exported nfpol into the PPO actor + obs normalizer."""
    lin = [m for m in algo.actor.net if isinstance(m, torch.nn.Linear)]
    assert len(lin) == len(pol["layers"])
    with torch.no_grad():
        for m, L in zip(lin, pol["layers"]):
            m.weight.copy_(torch.tensor(L["W"]))
            m.bias.copy_(torch.tensor(L["b"]))
    std = torch.tensor(pol["obs_std"], dtype=torch.float64)
    algo.obs_rms.mean = torch.tensor(pol["obs_mean"], dtype=torch.float64)
    algo.obs_rms.var = std ** 2 - algo.obs_rms.eps
    algo.obs_rms.count = 1e7


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=400)
    ap.add_argument("--warmup", type=int, default=40)
    a = ap.parse_args(argv)
    veh, rand, rpath = patch_vehicle()
    from train.ppo import PPO, load_config
    from train import export
    cfg = load_config(os.path.join(NF, "train", "config", "hover_ctbr_v3.yaml"))
    cfg["randomization"] = rpath   # absolute path: os.path.join(ROOT, abs) keeps it
    cfg["checkpoint_every"] = 25
    cfg.pop("init_from", None)
    last = os.path.join(OUT, "last.pt")
    resume = os.path.exists(last)
    if resume:
        cfg["init_from"] = last
    torch.set_num_threads(int(cfg.get("threads", 4)))
    algo = PPO(cfg, OUT)
    done = 0
    if resume:
        done = int(torch.load(last, map_location="cpu", weights_only=False).get("ft_iter", 0))
        print(f"resuming at iteration {done}", flush=True)
    else:
        with open(V3) as f:
            nfpol_to_state(json.load(f), algo)
        with torch.no_grad():
            algo.actor.log_std.fill_(math.log(0.1))
    log = open(os.path.join(OUT, "log.txt"), "a")
    while done < a.iters + a.warmup:
        frozen = done < a.warmup
        if frozen:
            snap = copy.deepcopy(algo.actor.state_dict())
        s = algo.iterate()
        if frozen:  # critic warm-up: undo any actor change
            algo.actor.load_state_dict(snap)
        done += 1
        if done % 10 == 0 or done == 1:
            line = (f"it {done:4d}{' (critic warm-up)' if frozen else ''} surv {s['survival']:.3f} "
                    f"ret {s['ep_ret']:7.2f} len {s['ep_len']:5.0f} std {s['std']:.3f} vf {s['vf']:.4f} "
                    f"kl {s['kl']:.4f}")
            print(line, flush=True)
            log.write(line + "\n"); log.flush()
        if done % 25 == 0 or done == a.iters + a.warmup:
            algo.save(last)
            st = torch.load(last, map_location="cpu", weights_only=False)
            st["ft_iter"] = done
            torch.save(st, last)
    out = export.export_checkpoint(last, os.path.join(OUT, "policy.nfpol.json"))
    print(f"exported {out}", flush=True)


if __name__ == "__main__":
    main()

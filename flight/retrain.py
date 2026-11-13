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

RESULT (2026-11-13): this approach does not work, and NF-3D does not use it.
Every fine-tune lost v3's hover precision (median hover error 3.7 cm -> 22-27 cm)
within ~60 actor iterations. The same happens when fine-tuning on neuroflight-fc's
own reference vehicle (NF3D_VEHICLE=reference), so it is the method, not the
airframe. Ruled out: curriculum level (fixed: init_level), observation-normalizer
drift (fixed: frozen), the nfpol->checkpoint->nfpol round trip (exact), the CAPS
smoothness loss (NF3D_CAPS=0: same), critic warm-up length (100 iterations: same).
A 10x lower learning rate is worse, because the fresh critic never fits. Most likely
cause: without v3's trained critic the advantage estimates are too noisy to hold a
cm-level hover. What NF-3D uses instead is flight/fly.py CollectiveScaled (v3
weights, collective thrust scaled to the airframe).

    python -m flight.retrain [--iters 400] [--warmup 40]
Writes runs/$NF3D_RUN (default nf3d_ft4)/{last.pt,policy.nfpol.json} under this repo.
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
OUT = os.path.join(ROOT, "runs", os.environ.get("NF3D_RUN", "nf3d_ft4"))


def patch_vehicle():
    """Make every neuroflight-fc module that loads the nominal vehicle see NF-3D."""
    sys.path.insert(0, NF)
    with open(os.path.join(ROOT, "out", "vehicle_%s.yaml" % os.environ.get("NF3D_VEHICLE", "nominal"))) as f:
        veh = yaml.safe_load(f)
    import sim.params, sim.env, sim.quadrotor, train.ppo, train.priv_env  # noqa: E401,F401
    for name, mod in list(sys.modules.items()):
        if (name.startswith("sim.") or name.startswith("train.")) and hasattr(mod, "load_nominal"):
            mod.load_nominal = lambda *a, **k: copy.deepcopy(veh)
    with open(os.path.join(NF, "config", "randomization.yaml")) as f:
        rand = yaml.safe_load(f)
    with open(os.path.join(ROOT, "out", "vehicle_pessimistic.yaml")) as f:
        pess = yaml.safe_load(f)
    # NF3D_WIDEN=1 widens mass/thrust so the pessimistic corner is in-distribution.
    # Run nf3d_ft2 did that and learned a compromise thrust bias: the policy has no
    # integral action, so it settled ~10 cm off target and failed the eval tolerance.
    # Default (0) keeps neuroflight-fc's +-15 % ranges around the NF-3D nominal.
    if os.environ.get("NF3D_WIDEN", "0") == "1":
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
    if os.environ.get("NF3D_LR"):   # PPO lr (adaptive between lr_min and 2.5x this)
        lr = float(os.environ["NF3D_LR"])
        cfg["ppo"].update(lr=lr, lr_max=2.5 * lr, lr_min=min(cfg["ppo"].get("lr_min", 1e-5), lr / 4))
    if os.environ.get("NF3D_CAPS", "1") == "0":   # diagnostic: drop the CAPS smoothness loss
        cfg["caps"] = {}
    cfg.pop("init_from", None)
    last = os.path.join(OUT, "last.pt")
    resume = os.path.exists(last)
    if resume:
        cfg["init_from"] = last
    cfg["threads"] = int(os.environ.get("NF3D_THREADS", cfg.get("threads", 4)))
    torch.set_num_threads(cfg["threads"])
    algo = PPO(cfg, OUT)
    # Keep v3's observation normalizer fixed. PPO keeps updating it from rollouts, and the
    # count we can give it from an nfpol (1e7) is swamped within ~80 iterations (131k
    # samples each). Run nf3d_ft3 showed the effect: return peaked during the critic
    # warm-up with the actor frozen, then fell as the normalizer drifted, and the policy
    # ended with a 22 cm hover error. The actor was trained against v3's statistics.
    if os.environ.get("NF3D_FREEZE_NORM", "1") == "1":
        algo.obs_rms.update = lambda *a, **k: None
    done = 0
    if resume:
        done = int(torch.load(last, map_location="cpu", weights_only=False).get("ft_iter", 0))
        print(f"resuming at iteration {done}", flush=True)
    else:
        with open(V3) as f:
            nfpol_to_state(json.load(f), algo)
        with torch.no_grad():
            algo.actor.log_std.fill_(math.log(0.1))
        # PPO only restores the curriculum level from a checkpoint (init_from). Starting
        # from an nfpol we must set it ourselves, or training restarts at the easiest
        # level (15 deg tilt, no throws) and v3's recovery behaviour decays. That is what
        # sank the first fine-tune (runs/nf3d_ft): return fell from ~180 to ~70.
        algo.curr.level = min(int(cfg.get("init_level", 0)), len(algo.curr.levels) - 1)
        algo.curr.apply(algo.env.cfg)
        print(f"curriculum level set to {algo.curr.level}", flush=True)
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
            line = (f"it {done:4d}{' (critic warm-up)' if frozen else ''} lvl {algo.curr.level} surv {s['survival']:.3f} "
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

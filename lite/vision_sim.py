"""NF-3D Lite v3: closed-loop vision simulation of the AprilTag behaviours.

Everything between the camera and the motors runs the real code or its tested twin:

  neuroflight-fc simulator (fw3 sensing: flow/ToF KF, KF-referenced Mahony), randomized vehicles
    -> synthetic OV2640 frames: the printed 36h11 tag warped into a 320x240 image from the
       drone's true pose, blurred, noised and JPEG-compressed like the XIAO stream, 15 fps
    -> lite/ground/vision.py TagDetector (OpenCV aruco + solvePnP), the laptop's code
    -> lite/ground/control.py TagController, the laptop's code
    -> CMD frame through lite/ground/link.py (cm/s, mrad/s quantisation), 150 ms total latency
       (stream + detection + Wi-Fi + UART), AUTO-mode handling of lite::Autopilot (clamps, stale
       commands, LAND)
    -> lite/autonomy.py Autopilot (twin of the firmware position hold) -> motors

Scenarios (gated; thresholds fixed before the run):
  follow_static   tag 2.0 m ahead, start offset up to +-0.4 m and +-25 deg yaw: after 8 s the
                  drone is within 0.12 m of the stand-off point (0.8 m in front) and faces the tag
                  within 8 deg, in every vehicle
  follow_moving   the tag walks sideways at 0.25 m/s for 4 s then stops: tracking error < 0.35 m
                  while moving, < 0.15 m 4 s after it stops, the tag never lost for > 1 s
  approach_land   approach to 0.5 m in front of the tag and land: lands within 0.15 m of that
                  point, touchdown < 0.6 m/s, disarmed, in every vehicle
  tag_removed     tag taken away at 4 s: the drone stops (speed < 0.15 m/s 1.5 s later) and stays
                  within 0.3 m
  search          tag 90 deg to the side, out of view: the drone turns, finds it and reaches the
                  stand-off point within 15 s

    python -m lite.vision_sim [--n 8]      -> lite/out/vision.json (+ lite/out/vision_frames.png)
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys

import cv2
import numpy as np
import torch

from . import autonomy as A
from .fly import OUT
from .ground import link
from .ground.control import ControlConfig, TagController
from .ground.vision import TAG_SIZE_M, Camera, TagDetector

FPS = 15.0
LATENCY_S = 0.15
CAM_OFFSET_B = np.array([0.045, 0.0, 0.02])   # camera on the nose arch, body FLU (m)
M_BC = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])  # body FLU -> OpenCV camera
CMD_V_MAX = None                                # from spec.yaml (the flight board's clamp)


# ------------------------------------------------------------------------------- rendering
class TagWorld:
    """A printed tag in the world: centre, face normal (horizontal), id."""

    def __init__(self, centre, normal, tag_id=0, size=TAG_SIZE_M):
        self.c = np.asarray(centre, float)
        n = np.asarray(normal, float)
        self.n = n / np.linalg.norm(n)
        self.id, self.s = tag_id, size
        up = np.array([0.0, 0.0, 1.0])
        self.r = np.cross(up, self.n)                     # tag "right" as seen from the front
        cells = 10                                        # 8 black-bordered cells + 1 white quiet cell each side
        px = 12
        mk = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), tag_id, 8 * px)
        self.canvas = np.full((cells * px, cells * px), 255, np.uint8)
        self.canvas[px:9 * px, px:9 * px] = mk
        # the white sheet around the tag: 1.6x the tag edge (an A5-ish print)
        self.sheet = 1.6 * size
        W = self.canvas.shape[0]
        self.src = np.array([[0, 0], [W, 0], [W, W], [0, W]], np.float32)

    def corners_world(self):
        h = self.s / 2 * 10 / 8                            # outer edge of the quiet zone
        up = np.array([0.0, 0.0, 1.0])
        return np.array([self.c + h * (-self.r + up), self.c + h * (self.r + up), self.c + h * (self.r - up),
                         self.c + h * (-self.r - up)])


def _background(rng, cam):
    """Low-frequency grey texture (a room), fixed per vehicle."""
    small = rng.integers(60, 200, size=(6, 8)).astype(np.uint8)
    bg = cv2.resize(small, (cam.width, cam.height), interpolation=cv2.INTER_CUBIC)
    return cv2.GaussianBlur(bg, (0, 0), 6)


def render(cam: Camera, tag: TagWorld | None, pos_w, R_wb, bg, rng, jpeg_q=70):
    img = bg.copy()
    if tag is not None:
        cam_w = pos_w + R_wb @ CAM_OFFSET_B
        P = tag.corners_world()
        pc = (M_BC @ (R_wb.T @ (P - cam_w).T)).T            # (4,3) in the camera frame
        facing = np.dot(tag.n, cam_w - tag.c) > 0
        if facing and (pc[:, 2] > 0.05).all():
            uv = (cam.K @ pc.T).T
            uv = (uv[:, :2] / uv[:, 2:3]).astype(np.float32)
            if np.abs(uv).max() < 5000:
                H = cv2.getPerspectiveTransform(tag.src, uv)
                warped = cv2.warpPerspective(tag.canvas, H, (cam.width, cam.height), flags=cv2.INTER_AREA,
                                             borderValue=0)
                mask = cv2.warpPerspective(np.full_like(tag.canvas, 255), H, (cam.width, cam.height),
                                           flags=cv2.INTER_LINEAR, borderValue=0)
                a = mask.astype(np.float32) / 255.0
                img = (img * (1 - a) + warped * a * 0.85 + 20 * a).astype(np.uint8)  # paper is not pure white
    img = cv2.GaussianBlur(img, (0, 0), 0.7)
    img = np.clip(img + rng.normal(0, 3.0, img.shape), 0, 255).astype(np.uint8)
    ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, jpeg_q])
    return cv2.imdecode(enc, cv2.IMREAD_GRAYSCALE)


# ------------------------------------------------------------------------------- the loop
def _start_env(ev, veh, n, seed, randomized, starts):
    """starts: list of (x, y, z, yaw) per env (repeated if shorter)."""
    env = A._hover_env(ev, veh, n, seed, randomized)
    q = env.quad
    n = env.n
    idx = torch.arange(n)
    pos = torch.tensor([starts[i % len(starts)][:3] for i in range(n)], dtype=torch.float32)
    yaw = torch.tensor([starts[i % len(starts)][3] for i in range(n)], dtype=torch.float32)
    quat = torch.stack((torch.cos(yaw / 2), torch.zeros(n), torch.zeros(n), torch.sin(yaw / 2)), -1)
    q.set_state(idx, pos=pos, vel=torch.zeros(n, 3), omega=torch.zeros(n, 3), quat=quat, motor=q.motor.clone())
    env.reinit_sensing(idx)
    env.cfg.oob_dist = 1e9
    env.quad.floor_z = -1e9
    return env


def run(ev, veh, spec, n, seed, secs, randomized, starts, tag_fn, cfg: ControlConfig, record=None):
    """tag_fn(t) -> TagWorld or None. Returns per-env traces (numpy)."""
    env = _start_env(ev, veh, n, seed, randomized, starts)
    n = env.n
    ap = A.Autopilot(env, veh, spec)
    st = {"landed": torch.zeros(n, dtype=torch.bool), "td_speed": torch.full((n,), float("nan"))}
    q, sample_accel = env.quad, env.quad.sample_accel

    def _accel_on_floor():
        a = sample_accel()
        L = st["landed"]
        if bool(L.any()):
            a[L] = a[L] - q.specific_force_b[L] + torch.tensor([0.0, 0.0, A.G])
        return a
    q.sample_accel = _accel_on_floor

    cam = Camera()
    det = TagDetector(cam)
    ctl = [TagController(copy.deepcopy(cfg)) for _ in range(n)]
    rngs = [np.random.default_rng(seed * 1000 + i) for i in range(n)]
    bgs = [_background(r, cam) for r in rngs]
    vmax = np.array(spec["sim"]["pos_hold"]["cmd_v_max"], float)
    pending = []                      # (release_t, env_index, Cmd)
    last_cmd_t = np.full(n, -1e9)
    v_head = np.zeros((n, 3))
    yaw_rate = np.zeros(n)
    steps = int(secs / env.control_dt)
    frame_every = int(round(1.0 / FPS / env.control_dt))
    tr = {"t": [], "pos": [], "yaw": [], "seen": [], "mode": [], "armed": []}
    seen = np.zeros(n, bool)
    for k in range(steps):
        t = k * env.control_dt
        if k % frame_every == 0:
            tag = tag_fn(t)
            P = q.pos.numpy().astype(float)
            R = q.R.numpy().astype(float)
            for i in range(n):
                img = render(cam, tag, P[i], R[i], bgs[i], rngs[i])
                poses = det.detect(img)
                seen[i] = len(poses) > 0
                cmd = ctl[i].update(t, poses)
                wire = link.Parser().feed(cmd.pack())[0]          # through the frame encoder / parser
                pending.append((t + LATENCY_S, i, wire))
                if record is not None and i == 0 and k % (frame_every * 15) == 0 and len(record) < 8:
                    record.append(img)
        # deliver commands whose latency has elapsed (the firmware's AUTO handling)
        keep = []
        for rel, i, c in pending:
            if rel > t:
                keep.append((rel, i, c))
                continue
            last_cmd_t[i] = t
            if c.op == link.OP_VEL:
                f, l, u, w = c.velocity_si()
                v_head[i] = np.clip([f, l, u], -vmax, vmax)
                yaw_rate[i] = float(np.clip(w, -1.0, 1.0))
            elif c.op == link.OP_LAND:
                ap.request_land(torch.tensor([i]))
                v_head[i] = 0.0
                yaw_rate[i] = 0.0
            else:                                                   # HOLD
                v_head[i] = 0.0
                yaw_rate[i] = 0.0
        pending = keep
        stale = (t - last_cmd_t) > 0.5
        v_head[stale] = 0.0
        yaw_rate[stale] = 0.0
        Ra = env._R_att()
        yaw_est = torch.atan2(Ra[:, 1, 0], Ra[:, 0, 0]).numpy()
        c_, s_ = np.cos(yaw_est), np.sin(yaw_est)
        vw = np.stack((c_ * v_head[:, 0] - s_ * v_head[:, 1], s_ * v_head[:, 0] + c_ * v_head[:, 1], v_head[:, 2]), -1)
        ap.v_cmd = torch.tensor(vw, dtype=torch.float32)
        ap.yaw_rate = torch.tensor(yaw_rate, dtype=torch.float32)
        env.step(ap())
        A._ground(env, st)
        if k % 10 == 0:
            Rt = q.R
            tr["t"].append(t)
            tr["pos"].append(q.pos.numpy().copy())
            tr["yaw"].append(torch.atan2(Rt[:, 1, 0], Rt[:, 0, 0]).numpy().copy())
            tr["seen"].append(seen.copy())
            tr["mode"].append(ap.mode.numpy().copy())
            tr["armed"].append(ap.armed.numpy().copy())
    out = {k: np.array(v) for k, v in tr.items()}
    out["td_speed"] = st["td_speed"].numpy()
    out["landed"] = st["landed"].numpy()
    qp = env.quad.params
    out["flyable"] = ((qp.thrust_max.sum(-1) / (qp.mass * A.G)) >= A.MIN_TW).numpy()
    return out


# ------------------------------------------------------------------------------- scenarios
TAG = TagWorld((2.0, 0.0, 1.0), (-1.0, 0.0, 0.0), 0)


def _standoff(tag, d):
    return tag.c + d * tag.n


def _facing_err(tr, tag, k):
    """Angle between the drone's heading and the direction to the tag (rad), per env, at sample k."""
    d = tag.c[None, :2] - tr["pos"][k][:, :2]
    want = np.arctan2(d[:, 1], d[:, 0])
    return np.abs((tr["yaw"][k] - want + np.pi) % (2 * np.pi) - np.pi)


def scenario_follow_static(ev, veh, spec, n, seed):
    # heading up to 20 deg off the line to the tag (the camera sees +-30 deg)
    pts = [(0.3, 0.4, 0.9, 20), (0.2, -0.4, 1.1, -20), (0.0, 0.0, 1.0, 0), (0.5, 0.3, 1.2, -15)]
    starts = [(x, y, z, math.atan2(TAG.c[1] - y, TAG.c[0] - x) + math.radians(d)) for x, y, z, d in pts]
    tr = run(ev, veh, spec, n, seed, 10.0, True, starts, lambda t: TAG, ControlConfig(task="follow"))
    goal = _standoff(TAG, 0.8)
    k8 = int(8.0 / 0.02)
    err = np.linalg.norm(tr["pos"][k8:] - goal[None, None, :], axis=-1).max(0)
    f = tr["flyable"]      # rows whose randomized motors can't lift the drone with margin: reported, not judged
    fac = max(_facing_err(tr, TAG, k)[f].max() for k in range(k8, len(tr["t"])))
    return {"err_after_8s_max_m": float(err[f].max()), "facing_err_max_deg": float(math.degrees(fac)),
            "seen_frac": float(tr["seen"][:, f].mean()), "unflyable": int((~f).sum())}


def scenario_follow_moving(ev, veh, spec, n, seed):
    def tag_fn(t):
        y = -0.25 * min(max(t - 3.0, 0.0), 4.0)      # walks 1 m to the right (-y) from t = 3 to 7 s
        return TagWorld((2.0, y, 1.0), (-1.0, 0.0, 0.0), 0)
    tr = run(ev, veh, spec, n, seed, 11.0, True, [(1.2, 0.0, 1.0, 0.0)], tag_fn, ControlConfig(task="follow"))
    t = tr["t"]
    goal = np.array([_standoff(tag_fn(x), 0.8) for x in t])
    e = np.linalg.norm(tr["pos"] - goal[:, None, :], axis=-1)
    moving = (t > 3.5) & (t < 7.0)
    after = t > 11.0 - 0.05 - 0.0
    after = t >= 10.9
    # longest run of frames without the tag
    unseen = ~tr["seen"]
    f = tr["flyable"]
    longest = 0.0
    for i in np.nonzero(f)[0]:
        run_, best = 0, 0
        for u in unseen[:, i]:
            run_ = run_ + 1 if u else 0
            best = max(best, run_)
        longest = max(longest, best * 0.02)
    return {"err_moving_max_m": float(e[moving][:, f].max()), "err_4s_after_stop_max_m": float(e[after][:, f].max()),
            "longest_unseen_s": longest, "unflyable": int((~f).sum())}


def scenario_approach_land(ev, veh, spec, n, seed):
    starts = [(0.0, 0.3, 1.0, math.radians(15)), (0.3, -0.3, 1.2, math.radians(-10)), (0.1, 0.0, 0.9, 0.0)]
    tr = run(ev, veh, spec, n, seed, 16.0, True, starts, lambda t: TAG, ControlConfig(task="land"))
    goal = _standoff(TAG, 0.5)
    land_err = np.linalg.norm(tr["pos"][-1][:, :2] - goal[None, :2], axis=-1)
    f = tr["flyable"]
    return {"land_err_max_m": float(land_err[f].max()), "touchdown_max": float(np.nan_to_num(tr["td_speed"][f], nan=9.9).max()),
            "landed": float(tr["landed"][f].mean()), "disarmed": float((~tr["armed"][-1][f]).mean()),
            "unflyable": int((~f).sum())}


def scenario_tag_removed(ev, veh, spec, n, seed):
    tr = run(ev, veh, spec, n, seed, 8.0, True, [(0.6, 0.2, 1.0, 0.0)], lambda t: TAG if t < 4.0 else None,
             ControlConfig(task="follow", search_after_s=1e9))
    t = tr["t"]
    k4, k55 = int(4.0 / 0.02), int(5.5 / 0.02)
    v = np.linalg.norm(np.diff(tr["pos"], axis=0), axis=-1) / 0.02
    drift = np.linalg.norm(tr["pos"][k4:, :, :2] - tr["pos"][k4][None, :, :2], axis=-1).max(0)
    f = tr["flyable"]
    return {"speed_1p5s_after_max": float(v[k55:][:, f].max()), "drift_max_m": float(drift[f].max()), "unflyable": int((~f).sum())}


def scenario_search(ev, veh, spec, n, seed):
    tag = TagWorld((0.3, 2.0, 1.0), (0.0, -1.0, 0.0), 0)    # 2 m to the drone's left, out of view
    tr = run(ev, veh, spec, n, seed, 16.0, True, [(0.3, 0.0, 1.0, 0.0)], lambda t: tag,
             ControlConfig(task="follow", search_after_s=1.0))
    goal = _standoff(tag, 0.8)
    e = np.linalg.norm(tr["pos"] - goal[None, None, :], axis=-1)
    k15 = int(15.0 / 0.02)
    f = tr["flyable"]
    return {"err_at_15s_max_m": float(e[k15:][:, f].max()), "unflyable": int((~f).sum())}


GATES = {
    "follow_static": lambda r: r["err_after_8s_max_m"] < 0.12 and r["facing_err_max_deg"] < 8.0,
    "follow_moving": lambda r: r["err_moving_max_m"] < 0.35 and r["err_4s_after_stop_max_m"] < 0.15 and r["longest_unseen_s"] <= 1.0,
    "approach_land": lambda r: r["land_err_max_m"] < 0.15 and r["touchdown_max"] < 0.6 and r["landed"] == 1.0 and r["disarmed"] == 1.0,
    "tag_removed": lambda r: r["speed_1p5s_after_max"] < 0.15 and r["drift_max_m"] < 0.3,
    "search": lambda r: r["err_at_15s_max_m"] < 0.15,
}


def main(argv=None):
    ap_ = argparse.ArgumentParser()
    ap_.add_argument("--n", type=int, default=8)
    ap_.add_argument("--seed", type=int, default=3)
    ap_.add_argument("--only", default="")
    ap_.add_argument("--corner", default="nominal,pessimistic")
    a = ap_.parse_args(argv)
    torch.set_num_threads(max(1, os.cpu_count() or 1))
    from .frame import load_spec
    spec = load_spec()
    checks = json.load(open(os.path.join(OUT, "checks.json")))
    ev = A._eval_mod()
    vehs = A._vehicles(spec, checks, ev)
    out, fails = {}, []
    scen = {"follow_static": scenario_follow_static, "follow_moving": scenario_follow_moving,
            "approach_land": scenario_approach_land, "tag_removed": scenario_tag_removed, "search": scenario_search}
    for corner in a.corner.split(","):
        for name, fn in scen.items():
            if a.only and name not in a.only.split(","):
                continue
            r = fn(ev, vehs[corner], spec, a.n, a.seed)
            out.setdefault(corner, {})[name] = r
            ok = GATES[name](r)
            print(f"[{corner}] {name}: {'PASS' if ok else 'FAIL'} {r}", flush=True)
            if not ok:
                fails.append(f"{corner} {name}: {r}")
    # a strip of what the camera sees, for the docs
    frames = []
    run(ev, vehs["nominal"], spec, 1, a.seed, 3.0, False, [(0.3, 0.3, 1.0, math.radians(20))], lambda t: TAG,
        ControlConfig(task="follow"), record=frames)
    if frames:
        cv2.imwrite(os.path.join(OUT, "vision_frames.png"), np.hstack(frames[:3]))
    out["failures"] = fails
    with open(os.path.join(OUT, "vision.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print("FAIL" if fails else "PASS", *fails, sep="\n  ")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

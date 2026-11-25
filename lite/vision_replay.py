"""Replay video of the NF-3D Lite vision behaviours (simulation) for the README.

    python -m lite.vision_replay [--out docs/media]       (needs ../neuroflight-fc, torch, opencv)

Runs two closed-loop episodes through lite.vision_sim.run, exactly the loop the gated scenarios
use (neuroflight-fc simulator, synthetic OV2640 frames, the laptop's TagDetector and TagController,
150 ms link latency, the firmware's position-hold twin), on the nominal vehicle:

  1. search + follow: the tag starts 90 deg to the left, out of view; the drone turns, finds it,
     takes up the 0.8 m stand-off, then follows it as it walks 0.8 m sideways
  2. approach + land: from an offset start, settle 0.5 m in front of the tag and land

and draws, per camera frame, what the drone sees (with the detector's corners) next to a top-down
map and the height trace. Writes vision-replay.{mp4,gif}. Every frame is labelled as simulation.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import tempfile
from pathlib import Path

import cv2
import numpy as np
import torch

from . import autonomy as A
from . import vision_sim as V
from .ground.control import ControlConfig

ROOT = Path(__file__).resolve().parents[1]
BG, PANEL, GRID, TXT, DIM = "#0d1117", "#161b22", "#30363d", "#e6edf3", "#8b949e"
C_DRONE, C_TAG, C_PATH, C_OK = "#f0883e", "#e6edf3", "#58a6ff", "#3fb950"


def episode(ev, veh, spec, secs, start, tag_fn, cfg):
    frames = []
    real_render = V.render

    def grab(cam, tag, pos_w, R_wb, bg, rng, jpeg_q=70):
        img = real_render(cam, tag, pos_w, R_wb, bg, rng, jpeg_q)
        frames.append(img)
        return img
    V.render = grab
    try:
        tr = V.run(ev, veh, spec, 1, 3, secs, False, [start], tag_fn, cfg)
    finally:
        V.render = real_render
    return frames, tr


def draw(episodes, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as Poly
    plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": TXT, "axes.labelcolor": DIM,
                         "xtick.color": DIM, "ytick.color": DIM, "axes.edgecolor": GRID})
    aruco = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    tmp = Path(tempfile.mkdtemp())
    f_idx = 0
    _, fr0, tr0, _, _ = episodes[0]
    fps = max(1, round(len(fr0) / tr0["t"][-1]))              # play in real time
    for title, frames, tr, tag_fn, note in episodes:
        t = tr["t"]
        P = tr["pos"][:, 0]
        yaw = tr["yaw"][:, 0]
        lo = np.minimum(P[:, :2].min(0), 0) - 0.5
        hi = P[:, :2].max(0) + 0.5
        tags = [tag_fn(x) for x in t]
        for tg in tags:
            if tg is not None:
                lo = np.minimum(lo, tg.c[:2] - 0.4)
                hi = np.maximum(hi, tg.c[:2] + 0.4)
        span = (hi - lo).max()
        mid = (hi + lo) / 2
        fig = plt.figure(figsize=(12.8, 6.4), dpi=100, facecolor=BG)
        ax_cam = fig.add_axes([0.03, 0.19, 0.47, 0.61])
        ax_map = fig.add_axes([0.56, 0.30, 0.41, 0.50])
        ax_z = fig.add_axes([0.56, 0.10, 0.41, 0.13])
        fig.text(0.03, 0.92, title, fontsize=17, weight="bold", color=TXT)
        fig.text(0.03, 0.875, "SIMULATION  ·  neuroflight-fc simulator, synthetic OV2640 frames (blur, noise, JPEG), the laptop's real "
                 "TagDetector + TagController, 150 ms link latency", fontsize=9, color=DIM)
        fig.text(0.03, 0.115, note, fontsize=10, color=DIM, wrap=True)
        ax_cam.set_title("onboard camera (320×240, 15 fps)", fontsize=10, color=DIM, loc="left")
        ax_map.set_title("top-down", fontsize=10, color=DIM, loc="left")
        for a in (ax_map, ax_z):
            a.set_facecolor(PANEL)
            a.grid(color=GRID, lw=0.6)
        ext = hi - lo
        aspect = 0.50 * 6.4 / (0.41 * 12.8)                  # map axes height / width
        w = max(ext[0], ext[1] / aspect)
        ax_map.set_xlim(mid[0] - w / 2, mid[0] + w / 2)
        ax_map.set_ylim(mid[1] - w * aspect / 2, mid[1] + w * aspect / 2)
        ax_map.set_aspect("equal", adjustable="box")
        span = w
        ax_map.set_xlabel("x  m", fontsize=8)
        ax_z.set_xlim(0, t[-1])
        ax_z.set_ylim(0, max(1.4, P[:, 2].max() * 1.15))
        ax_z.set_ylabel("height m", fontsize=8)
        ax_z.set_xlabel("time  s", fontsize=8)
        ax_z.plot(t, P[:, 2], color=GRID, lw=0.8)
        zl, = ax_z.plot([], [], color=C_PATH, lw=1.6)
        path, = ax_map.plot([], [], color=C_PATH, lw=1.8)
        tag_ln, = ax_map.plot([], [], color=C_TAG, lw=4, solid_capstyle="butt")
        tag_nm, = ax_map.plot([], [], color=C_TAG, lw=0.8, ls=":")
        drone, = ax_map.plot([], [], "o", ms=11, color=C_DRONE, mec=BG, mew=1.5, zorder=6)
        head, = ax_map.plot([], [], color=C_DRONE, lw=2.5, zorder=5)
        fov = Poly(np.zeros((3, 2)), closed=True, fc=C_DRONE, ec="none", alpha=0.12, zorder=4)
        ax_map.add_patch(fov)
        im = ax_cam.imshow(np.zeros((240, 320, 3), np.uint8))
        ax_cam.set_xticks([])
        ax_cam.set_yticks([])
        for s_ in ax_cam.spines.values():
            s_.set_color(GRID)
        status = ax_cam.text(8, 18, "", fontsize=11, color=C_OK, weight="bold", family="DejaVu Sans Mono")
        clock = ax_cam.text(312, 232, "", fontsize=10, color=TXT, ha="right", family="DejaVu Sans Mono")
        n_tr = len(t)
        dtf = t[-1] / len(frames)                             # camera frame period in sim time
        for k, img in enumerate(frames):
            tc = k * dtf
            j = min(int(round(tc / (t[1] - t[0]))), n_tr - 1)
            rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB) if img.ndim == 2 else img[..., ::-1].copy()
            gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray)
            if ids is not None and bool(tr["seen"][j, 0]):
                for c in corners:
                    cv2.polylines(rgb, [c.reshape(-1, 1, 2).astype(np.int32)], True, (63, 185, 80), 2, cv2.LINE_AA)
            im.set_data(rgb)
            armed = bool(tr["armed"][j, 0])
            landed = P[j, 2] <= 0.005
            if landed and not armed:
                st, cc = "LANDED · DISARMED", TXT
            elif int(tr["mode"][j, 0]) == A.MODE_LAND:
                st, cc = "LAND: DESCENDING 0.4 m/s", "#d2a8ff"
            elif bool(tr["seen"][j, 0]):
                st, cc = "TAG LOCKED", C_OK
            else:
                st, cc = "SEARCHING", C_DRONE
            status.set_text(st)
            status.set_color(cc)
            clock.set_text(f"t = {tc:5.1f} s")
            path.set_data(P[:j + 1, 0], P[:j + 1, 1])
            zl.set_data(t[:j + 1], P[:j + 1, 2])
            x, y, h = P[j, 0], P[j, 1], yaw[j]
            u, v = np.array([math.cos(h), math.sin(h)]), np.array([-math.sin(h), math.cos(h)])
            s = span * 0.035
            drone.set_data([x], [y])
            head.set_data([x, x + 3 * s * u[0]], [y, y + 3 * s * u[1]])
            r = span * 0.35
            hf = math.radians(31)
            fov.set_xy(np.array([[x, y], [x, y] + r * (math.cos(hf) * u + math.sin(hf) * v),
                                 [x, y] + r * (math.cos(hf) * u - math.sin(hf) * v)]))
            tg = tags[j]
            if tg is not None:
                a_, b_ = tg.c[:2] - tg.r[:2] * 0.25, tg.c[:2] + tg.r[:2] * 0.25
                tag_ln.set_data([a_[0], b_[0]], [a_[1], b_[1]])
                tag_nm.set_data([tg.c[0], tg.c[0] + 0.8 * tg.n[0]], [tg.c[1], tg.c[1] + 0.8 * tg.n[1]])
            fig.savefig(tmp / f"f{f_idx:05d}.png", facecolor=BG)
            f_idx += 1
        for _ in range(int(1.2 * fps)):                       # hold the last frame
            fig.savefig(tmp / f"f{f_idx:05d}.png", facecolor=BG)
            f_idx += 1
        plt.close(fig)
    out.mkdir(parents=True, exist_ok=True)
    mp4, gif = out / "vision-replay.mp4", out / "vision-replay.gif"
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(fps), "-i", str(tmp / "f%05d.png"), "-c:v", "libx264",
                    "-crf", "24", "-preset", "slow", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)], check=True)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", "2", "-t", "14", "-i", str(mp4), "-vf",
                    "fps=8,scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];"
                    "[b][p]paletteuse=dither=none:diff_mode=rectangle", str(gif)], check=True)
    return mp4, gif


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "media"))
    a = ap.parse_args(argv)
    import json
    torch.set_num_threads(max(1, os.cpu_count() or 1))
    from .frame import load_spec
    from .fly import OUT
    spec = load_spec()
    checks = json.load(open(os.path.join(OUT, "checks.json")))
    ev = A._eval_mod()
    veh = A._vehicles(spec, checks, ev)["nominal"]

    def walk(t):
        x = 0.3 + 0.2 * min(max(t - 10.0, 0.0), 4.0)          # walks 0.8 m along +x from t = 10 to 14 s
        return V.TagWorld((x, 2.0, 1.0), (0.0, -1.0, 0.0), 0)
    fr1, tr1 = episode(ev, veh, spec, 18.0, (0.3, 0.0, 1.0, 0.0), walk, ControlConfig(task="follow", search_after_s=1.0))
    fr2, tr2 = episode(ev, veh, spec, 14.0, (0.0, 0.3, 1.0, math.radians(15)), lambda t: V.TAG, ControlConfig(task="land"))
    eps = [("NF-3D Lite: search, lock on, follow", fr1, tr1, walk,
            "Tag starts 90° to the left, out of view. At t = 10 s it walks 0.8 m sideways and the drone follows at 0.8 m."),
           ("NF-3D Lite: approach and land on a tag", fr2, tr2, lambda t: V.TAG,
            "Settle 0.5 m in front of the tag, square to it, then LAND: descend at 0.4 m/s and disarm on the floor.")]
    for f in draw(eps, Path(a.out)):
        print(f, f"{f.stat().st_size / 1e6:.2f} MB")


if __name__ == "__main__":
    main()

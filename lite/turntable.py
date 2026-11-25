"""Turntable of the NF-3D Lite detailed model for the README (pyvista, off-screen).

    xvfb-run -a python -m lite.turntable [--out docs/media] [--frames 96]

Renders the 222-part detailed model from lite/detail.py (the one lite/audit.py checks: no overlaps,
no floating parts) with its own part colours, props turning in their real directions, and writes
lite-turntable.{mp4,gif} plus a hero still lite-hero.png.
"""
import argparse
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import pyvista as pv

from .detail import build_detail
from .frame import load_spec, motor_xy

pv.OFF_SCREEN = True
ROOT = Path(__file__).resolve().parents[1]


def to_mesh(part, tol=0.03, ang=0.2):
    verts, tris = part.tessellate(tol, ang)
    if not tris:
        return None
    V = np.array([[v.X, v.Y, v.Z] for v in verts])
    return pv.PolyData(V, np.hstack([np.full((len(tris), 1), 3), np.array(tris)]).ravel())


def hexrgb(c):
    return ((c >> 16) & 255) / 255, ((c >> 8) & 255) / 255, (c & 255) / 255


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "media"))
    ap.add_argument("--frames", type=int, default=96)
    ap.add_argument("--size", default="1280x880")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    W, H = map(int, a.size.split("x"))
    spec = load_spec()
    parts = [p for p in build_detail(spec, with_guard=False) if p.group != "sweep"]
    pl = pv.Plotter(off_screen=True, window_size=(W, H), lighting="none")
    pl.set_background("#f6f8fa", top="#dde3ea")
    pl.enable_anti_aliasing("ssaa")
    pl.add_light(pv.Light(position=(-120, -160, 260), focal_point=(0, 0, 0), intensity=0.85, light_type="scene light"))
    pl.add_light(pv.Light(position=(160, 120, 120), focal_point=(0, 0, 0), intensity=0.35, light_type="scene light"))
    pl.add_light(pv.Light(light_type="headlight", intensity=0.35))
    props = []
    zmin = 1e9
    for p in parts:
        m = to_mesh(p.part)
        if m is None:
            continue
        zmin = min(zmin, m.bounds[4])
        act = pl.add_mesh(m, color=hexrgb(p.color), smooth_shading=True, split_sharp_edges=True,
                          specular=0.25 + 0.6 * p.metal, specular_power=12 + 40 * p.metal, ambient=0.18,
                          diffuse=0.85, opacity=p.opacity)
        if p.name.startswith("Prop "):
            i = int(p.name.split()[1])
            props.append((act, i, "CCW" in p.name))
    # soft ground disc so the model is not floating in space
    floor = pv.Disc(center=(0, 0, zmin - 0.3), inner=0, outer=1500, c_res=160)
    pl.add_mesh(floor, color="#e9edf1", ambient=0.6, diffuse=0.4, specular=0.0)
    xy = motor_xy(spec)
    tmp = Path(tempfile.mkdtemp())
    focal = np.array([0.0, 0.0, 14.0])
    R, el = 185.0, 100.0
    for k in range(a.frames):
        az = 2 * math.pi * k / a.frames + math.radians(215)
        for act, i, ccw in props:
            x, y = xy[i]
            ang = (-1 if not ccw else 1) * 23.0 * k
            act.orientation = (0, 0, 0)
            act.origin = (x, y, 0)
            act.rotate_z(ang)
        pl.camera.focal_point = focal
        pl.camera.position = focal + np.array([R * math.cos(az), R * math.sin(az), el])
        pl.camera.up = (0, 0, 1)
        pl.camera.view_angle = 30
        pl.render()
        pl.screenshot(str(tmp / f"f{k:03d}.png"))
    pl.close()
    mp4, gif = out / "lite-turntable.mp4", out / "lite-turntable.gif"
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", "24", "-i", str(tmp / "f%03d.png"), "-c:v", "libx264",
                    "-crf", "21", "-preset", "slow", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)], check=True)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", "14", "-i", str(tmp / "f%03d.png"), "-vf",
                    "scale=680:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=160:stats_mode=diff[p];"
                    "[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle", "-loop", "0", str(gif)], check=True)
    subprocess.run(["cp", str(tmp / f"f{a.frames // 8:03d}.png"), str(out / "lite-hero.png")], check=True)
    board_still(parts, out / "lite-board.png")
    for f in (mp4, gif):
        print(f, f"{f.stat().st_size / 1e6:.2f} MB")


def board_still(parts, path, size=(1400, 1000)):
    """Close-up of the flight board as assembled: milled FR4, parts, modules (avionics group only)."""
    pl = pv.Plotter(off_screen=True, window_size=size, lighting="none")
    pl.set_background("#f6f8fa", top="#dde3ea")
    pl.enable_anti_aliasing("ssaa")
    pl.add_light(pv.Light(position=(-80, -120, 200), focal_point=(0, 0, 0), intensity=0.9, light_type="scene light"))
    pl.add_light(pv.Light(position=(120, 80, 90), focal_point=(0, 0, 0), intensity=0.3, light_type="scene light"))
    pl.add_light(pv.Light(light_type="headlight", intensity=0.3))
    pts = []
    for p in parts:
        if p.group != "avionics":
            continue
        m = to_mesh(p.part, tol=0.01, ang=0.15)
        if m is None:
            continue
        pts.append(m.center)
        pl.add_mesh(m, color=hexrgb(p.color), smooth_shading=True, split_sharp_edges=True,
                    specular=0.25 + 0.6 * p.metal, specular_power=12 + 40 * p.metal, ambient=0.18, diffuse=0.85)
    c = np.mean(pts, axis=0)
    pl.camera.focal_point = c
    pl.camera.position = c + np.array([-55.0, -80.0, 95.0])
    pl.camera.up = (0, 0, 1)
    pl.camera.view_angle = 30
    pl.render()
    pl.screenshot(str(path))
    pl.close()


if __name__ == "__main__":
    main()

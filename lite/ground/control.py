"""Tag-relative flight logic for the laptop: follow, approach-and-land, search.

Runs once per camera frame (~15 Hz) and returns the CMD frame to send. The flight board, in
AUTO, turns it into velocity / yaw-rate setpoints for its position-hold loop (lite/autonomy.hpp
Autopilot), so this is a slow outer loop: it only has to bring a target point to zero.

The same TagController drives the real drone (lite/ground/station.py) and the closed-loop
vision simulation (lite/vision_sim.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from . import link
from .vision import TagPose


@dataclass
class ControlConfig:
    task: str = "follow"          # follow | land
    target_id: int | None = None  # None: the nearest tag
    dist: float = 0.8             # m, follow stand-off in front of the tag
    land_dist: float = 0.5        # m, land this far in front of the tag
    kp_xy: float = 1.0            # 1/s, target error -> velocity
    kp_z: float = 0.8
    kp_yaw: float = 1.2           # 1/s, bearing -> yaw rate
    v_max: float = 0.5            # m/s (the flight board also limits to cmd_v_max)
    vz_max: float = 0.3
    yaw_max: float = 0.8          # rad/s
    filt_tau: float = 0.2         # s, low-pass on the target point (detections are noisy)
    lost_hold_s: float = 0.4      # no tag for this long: stop and hold
    search_after_s: float = 3.0   # ... for this long: turn in place to look for it (follow only)
    search_yaw: float = 0.5       # rad/s
    land_tol: float = 0.12        # m, within this of the land point ...
    land_yaw_tol: float = 0.10    # rad, ... and facing the tag ...
    land_settle_s: float = 1.0    # ... for this long: LAND
    align: bool = True            # use the tag's face normal (fly to face it squarely)


class TagController:
    def __init__(self, cfg: ControlConfig | None = None):
        self.c = cfg or ControlConfig()
        self.seq = 0
        self.last_seen = -1e9
        self.target = None           # filtered (fwd, left, up) of the goal point
        self.bear = 0.0
        self.t_prev = None
        self.settle = 0.0
        self.landing = False
        self.status = "waiting for tag"
        self.last_v = (0.0, 0.0, 0.0, 0.0)

    def _pick(self, poses: list[TagPose]) -> TagPose | None:
        if self.c.target_id is not None:
            poses = [p for p in poses if p.tag_id == self.c.target_id]
        return min(poses, key=lambda p: p.t_cam[2]) if poses else None

    def update(self, t: float, poses: list[TagPose]) -> link.Cmd:
        c = self.c
        dt = 0.0 if self.t_prev is None else max(0.0, t - self.t_prev)
        self.t_prev = t
        self.seq = (self.seq + 1) & 0xFFFF
        if self.landing:
            self.status = "landing"
            return link.Cmd(link.OP_LAND, seq=self.seq)
        p = self._pick(poses)
        if p is not None:
            d = c.dist if c.task == "follow" else c.land_dist
            if c.align:
                goal = p.standoff_point(d)
            else:
                f, l, u = p.body()
                r = math.hypot(f, l)
                goal = (f - d * f / r, l - d * l / r, u)
            fresh = self.target is None or t - self.last_seen > c.lost_hold_s
            a = 1.0 if fresh else min(1.0, dt / (c.filt_tau + dt))
            self.target = goal if fresh else tuple(o + a * (g - o) for o, g in zip(self.target, goal))
            self.bear = p.bearing() if fresh else self.bear + a * (p.bearing() - self.bear)
            self.last_seen = t
        elif self.target is not None:
            # no fix this frame: the goal moves the other way as the drone flies (dead-reckon on
            # the last command until the next fix or lost_hold_s)
            vf0, vl0, vu0, wz0 = self.last_v
            self.target = (self.target[0] - vf0 * dt, self.target[1] - vl0 * dt, self.target[2] - vu0 * dt)
            self.bear -= wz0 * dt
        age = t - self.last_seen
        if self.target is None or age > c.lost_hold_s:
            self.settle = 0.0
            self.last_v = (0.0, 0.0, 0.0, 0.0)
            if c.task == "follow" and age > c.search_after_s:
                self.status = "searching"
                turn = c.search_yaw if self.bear >= 0.0 else -c.search_yaw   # towards where it was last seen
                return link.Cmd.velocity(0.0, 0.0, 0.0, turn, self.seq)
            self.status = "tag lost: holding" if self.target is not None else "waiting for tag"
            return link.Cmd(link.OP_HOLD, seq=self.seq)
        f, l, u = self.target
        vf, vl = c.kp_xy * f, c.kp_xy * l
        n = math.hypot(vf, vl)
        if n > c.v_max:
            vf, vl = vf * c.v_max / n, vl * c.v_max / n
        vu = max(-c.vz_max, min(c.vz_max, c.kp_z * u))
        wz = max(-c.yaw_max, min(c.yaw_max, c.kp_yaw * self.bear))
        err = math.hypot(f, l)
        if c.task == "land":
            ok = err < c.land_tol and abs(self.bear) < c.land_yaw_tol and abs(u) < 0.15
            self.settle = self.settle + dt if ok else 0.0
            if self.settle >= c.land_settle_s:
                self.landing = True
                self.status = "landing"
                return link.Cmd(link.OP_LAND, seq=self.seq)
            self.status = f"approach: {err:.2f} m to the land point"
        else:
            self.status = f"following: {err:.2f} m off the stand-off point"
        self.last_v = (vf, vl, vu, wz)
        return link.Cmd.velocity(vf, vl, vu, wz, self.seq)

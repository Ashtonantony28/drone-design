"""NF-3D Lite v3 autonomy: position hold on optical flow + ToF, flight-mode failsafes, simulated.

The v3 Lite carries a PMW3901 optical-flow sensor and a VL53L1X time-of-flight sensor
(read by the XIAO companion board and passed to the flight board), so the firmware runs
neuroflight-fc's flow/ToF Kalman filter (nf::TranslationKf) and the KF-referenced Mahony
attitude filter: exactly the "fw3" sensing path the neuroflight-fc simulator models.

This file holds the Python twin of the firmware's PositionHold law (lite_fc
PositionHold, checked bit-for-bit-ish by lite/firmware/tests/pos_fixture.hpp) and the
flight-mode logic, and flies them in the neuroflight-fc simulator:

  modes     POS_HOLD   sticks / companion = velocity commands; zero = hold position + height
            ALT_HOLD   flow lost: hold height, attitude level (the pilot steers with tilt)
            LAND       range lost, LAND command, or controller link lost in AUTO: descend at
                       0.4 m/s on the KF vertical velocity, disarm when the ground stops it
  law       hold point ("carrot", moved by velocity commands, leashed 0.3 m) -> position P +
            command feed-forward -> velocity PI -> acceleration -> tilt + collective -> angle mode
            (the angle-mode inner loop of lite/fly.py, unchanged)
  landing   disarm when the controller asks < 60 % hover thrust for 0.5 s (the floor holds it up);
            with the height sensor gone, after a timed descent from the last known height

Tests (gated, thresholds declared before the run). Randomized vehicles whose motors cannot lift
them with margin (T/W < 1.15, which the pessimistic corner plus randomization produces) are
counted and reported, not judged.
  A  neuroflight-fc eval scenarios with the fw3 estimator: hover_1m, tilt_60, throw, each
     nominal + randomized + every randomization extreme; target (0, 0, 2) m, success = no
     crash, mean position error < 0.10 m over the last 0.5 s, tilt < 10 deg. Gate on the
     nominal / randomized groups: >= 95 % (nominal corner, 5 s), >= 90 % (pessimistic corner,
     T/W 1.27, 8 s to recover); the single-parameter extremes are reported.
  B  1 m position step: overshoot < 20 %, within 0.10 m by 3.0 s.
  C  20 s hold, randomized vehicles: RMS horizontal error < 0.10 m, height RMS < 0.05 m.
  D  velocity command 0.5 m/s for 3 s: speed error < 0.1 m/s after 1.5 s.
  E  flow dropout at 2 s -> ALT_HOLD: no crash, height within 0.20 m for 5 s.
  F  range dropout at 2 s -> LAND: touchdown < 1.0 m/s, disarmed within 2 s of touchdown and 12 s overall
     (the blind-land timeout bound; the touchdown time depends on the accelerometer bias).
  G  LAND command at 1.5 m: touchdown < 0.6 m/s, disarmed within 8 s and 2 s of touchdown.

    python -m lite.autonomy [--n 256]      -> lite/out/autonomy.json
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys

import torch

from .fly import NF, OUT, lite_vehicle

if NF not in sys.path:
    sys.path.insert(0, NF)

G = 9.81
MIN_TW = 1.15
MODE_POS, MODE_ALT, MODE_LAND, MODE_OFF = 0, 1, 2, 3


class PosHoldLaw:
    """Batched twin of lite::PositionHold (lite_fc/src/lite/autonomy.hpp). Float32 throughout.

    step() takes the KF estimate (world ENU), the attitude yaw, the setpoint and the
    sensor validity, and returns roll/pitch setpoints (rad, FLU, + roll = right side down,
    + pitch = nose down) and the collective before tilt compensation (N).
    """

    def __init__(self, n: int, cfg: dict, hover_n: float, dt: float):
        f = lambda v: torch.tensor(v, dtype=torch.float32)
        self.kp_pos, self.kp_vel, self.ki_vel = f(cfg["kp_pos"]), f(cfg["kp_vel"]), f(cfg["ki_vel"])
        self.i_lim, self.v_max = f(cfg["i_limit"]), f(cfg["v_max"])
        self.a_max_xy, self.max_tilt = float(cfg["a_max_xy"]), float(cfg["max_tilt_rad"])
        self.hover_n, self.dt, self.n = float(hover_n), float(dt), n
        self.integ = torch.zeros(n, 3)
        self.p_sp = torch.zeros(n, 3)

    def reset(self, idx, p):
        self.integ[idx] = 0.0
        self.p_sp[idx] = p[idx]

    XY_COAST_S = 0.5     # keep controlling x/y on the KF prediction through flow gaps this short
    LAND_SPEED = 0.4     # m/s
    LEASH = 0.3          # m, hold point at most this far from the drone, per axis
    XY_FORGET_S = 1.0    # after this long without flow, the hold point is re-taken where the drone is

    def step(self, p, v, yaw, v_cmd_in, mode, flow_age, z_valid):
        """v_cmd_in: (n,3) world velocity command (sticks / companion), 0 = hold.
        mode: (n,) MODE_*; flow_age: (n,) s since the last accepted flow update.
        Returns roll_sp, pitch_sp, thrust_n."""
        xy_valid = (flow_age < self.XY_COAST_S) & z_valid
        dt = self.dt
        land = mode == MODE_LAND
        cmd = v_cmd_in.clone()
        cmd[:, 2] = torch.where(land, torch.full_like(cmd[:, 2], -self.LAND_SPEED), cmd[:, 2])
        xy_on = ((mode == MODE_POS) | land) & xy_valid
        z_on = z_valid | land
        mask = torch.stack((xy_on, xy_on, z_on), -1)
        # setpoint ("carrot"): a velocity command moves the hold point at that speed and the
        # position loop keeps the drone on it (feed-forward + P), so small or slow commands never
        # drop an axis into open-loop velocity control. While commanded, the point stays within
        # LEASH of the drone (no wind-up when the drone cannot keep up); with no command it stays
        # put (a knock or a throw comes back); where the axis is not controlled it follows the drone
        forget = torch.stack(((flow_age > self.XY_FORGET_S) | ~z_valid,) * 2 + (~z_on,), -1)
        moving = cmd.abs() > 1e-3
        p_sp = self.p_sp + cmd * dt
        p_sp = torch.where(moving, p + (p_sp - p).clamp(-self.LEASH, self.LEASH), p_sp)
        self.p_sp = torch.where(forget, p, p_sp)
        v_d = cmd + self.kp_pos * (self.p_sp - p)
        vxy = v_d[:, :2]
        nxy = vxy.norm(dim=-1, keepdim=True)
        vxy = vxy * torch.clamp(self.v_max[0] / nxy.clamp_min(1e-6), max=1.0)
        vz = v_d[:, 2].clamp(-self.v_max[2], self.v_max[2])
        v_d = torch.cat((vxy, vz.unsqueeze(-1)), -1)
        e_v = v_d - v
        self.integ = torch.where(mask, (self.integ + self.ki_vel * e_v * dt).clamp(-self.i_lim, self.i_lim),
                                 self.integ)   # z off (height unknown, not landing): keep the trim
        self.integ[:, :2] = torch.where(mask[:, :2], self.integ[:, :2], torch.zeros_like(self.integ[:, :2]))
        a = torch.where(mask, self.kp_vel * e_v + self.integ, torch.zeros_like(e_v))
        axy = a[:, :2]
        na = axy.norm(dim=-1, keepdim=True)
        axy = axy * torch.clamp(self.a_max_xy / na.clamp_min(1e-6), max=1.0)
        az = a[:, 2].clamp(-0.5 * G, 0.8 * G)
        # where height is unknown and not landing: hold the hover collective (pilot's problem)
        c, s = torch.cos(yaw), torch.sin(yaw)
        af = c * axy[:, 0] + s * axy[:, 1]
        al = -s * axy[:, 0] + c * axy[:, 1]
        gz = G + az
        nrm = torch.sqrt(af * af + al * al + gz * gz)
        dl = al / nrm
        df = af / nrm
        dz = gz / nrm
        roll = torch.asin((-dl).clamp(-1, 1)).clamp(-self.max_tilt, self.max_tilt)
        pitch = torch.atan2(df, dz).clamp(-self.max_tilt, self.max_tilt)
        thrust = self.hover_n * gz / G
        return roll, pitch, thrust


class AngleLaw:
    """lite::angle_mode on the firmware attitude estimate, with per-row setpoints (fly.AngleMode
    generalised): q_sp = Rz(yaw_est) * from_euler(roll, pitch, 0)."""

    def __init__(self, env, veh: dict, kp):
        self.env = env
        self.kp = torch.tensor([kp[0], kp[1]], dtype=torch.float32)
        self.rmax = torch.tensor(veh["max_body_rate"], dtype=torch.float32)
        self.fmax = 4 * veh["thrust_max"]

    def __call__(self, roll, pitch, yaw_rate, thrust):
        R = self.env._R_att()
        b3 = R[:, :, 2]
        n = b3.shape[0]
        yaw = torch.atan2(R[:, 1, 0], R[:, 0, 0])
        cr, sr, cp, sp = torch.cos(roll), torch.sin(roll), torch.cos(pitch), torch.sin(pitch)
        zb = torch.stack((cr * sp, -sr, cr * cp), -1)                     # Ry(pitch) Rx(roll) e_z
        cy, sy = torch.cos(yaw), torch.sin(yaw)
        d3 = torch.stack((cy * zb[:, 0] - sy * zb[:, 1], sy * zb[:, 0] + cy * zb[:, 1], zb[:, 2]), -1)
        cvec = torch.cross(b3, d3, dim=-1)
        s = cvec.norm(dim=-1)
        ang = torch.atan2(s, (b3 * d3).sum(-1))
        axis = torch.where(s.unsqueeze(-1) > 1e-6, cvec / s.clamp_min(1e-6).unsqueeze(-1), R[:, :, 0])
        e_b = (R.transpose(1, 2) @ (axis * ang.unsqueeze(-1)).unsqueeze(-1)).squeeze(-1)
        rate = torch.zeros(n, 3)
        rate[:, :2] = self.kp * e_b[:, :2]
        rate[:, 2] = yaw_rate
        rate = torch.maximum(torch.minimum(rate, self.rmax), -self.rmax)
        cos_t = b3[:, 2].clamp(0.5, 1.0)
        F = torch.where(b3[:, 2] > 0, thrust / cos_t, 0.3 * thrust).clamp(0.0, self.fmax)
        a0 = F / self.fmax * 2 - 1
        return torch.cat((a0.unsqueeze(-1), rate / self.rmax), -1).clamp(-1, 1)


class Autopilot:
    """Mode logic + PosHoldLaw + AngleLaw: the firmware's v3 outer loop (lite::Autopilot)."""

    LAND_THRUST_FRAC = 0.6      # disarm when the controller asks for < 60 % of hover ...
    LAND_DETECT_S = 0.5         # ... for this long (the ground is holding the drone up)
    # Blind landing (height lost): the KF vertical velocity is dead-reckoned and freezes on the
    # floor, so the thrust detector may never fire. The KF velocity error frozen in at the loss
    # (up to ~0.2 m/s) makes the real descent 0.2-0.6 m/s, so disarm only after the descent from
    # the last known height at half speed: h / 0.4 * 2 + 1 s.
    BLIND_FACTOR, BLIND_MARGIN_S = 2.0, 1.0

    def __init__(self, env, veh, spec):
        self.env, self.veh = env, veh
        n = env.n
        self.dt = env.control_dt
        self.hover_n = veh["mass"] * veh["gravity"]      # firmware: nominal mass from vehicle_params
        self.law = PosHoldLaw(n, spec["sim"]["pos_hold"], self.hover_n, self.dt)
        self.angle = AngleLaw(env, veh, spec["sim"]["angle_kp"])
        self.mode = torch.full((n,), MODE_POS, dtype=torch.long)
        self.v_cmd = torch.zeros(n, 3)
        self.yaw_rate = torch.zeros(n)
        self.armed = torch.ones(n, dtype=torch.bool)
        self.low_t = torch.zeros(n)
        self.h_last = env.kf.p[:, 2].clone()
        self.blind_t = torch.zeros(n)
        self.law.reset(torch.arange(n), env.kf.p)

    def request_land(self, idx):
        self.mode[idx] = MODE_LAND

    def __call__(self, obs=None):
        kf = self.env.kf
        p, v = kf.p, kf.v
        z_valid = kf.height_age < 0.5
        xy_valid = (kf.flow_age < 0.5) & z_valid
        # failsafes: flow lost (0.5 s) -> ALT_HOLD; height lost (0.5 s) -> LAND (latched)
        self.mode = torch.where((self.mode == MODE_POS) & ~xy_valid & z_valid, torch.full_like(self.mode, MODE_ALT), self.mode)
        self.mode = torch.where((self.mode != MODE_OFF) & ~z_valid, torch.full_like(self.mode, MODE_LAND), self.mode)
        self.mode = torch.where((self.mode == MODE_ALT) & xy_valid, torch.full_like(self.mode, MODE_POS), self.mode)
        R = self.env._R_att()
        yaw = torch.atan2(R[:, 1, 0], R[:, 0, 0])
        roll, pitch, thrust = self.law.step(p, v, yaw, self.v_cmd, self.mode, kf.flow_age, z_valid)
        # landing detector
        low = (self.mode == MODE_LAND) & (thrust < self.LAND_THRUST_FRAC * self.hover_n)
        self.low_t = torch.where(low, self.low_t + self.dt, torch.zeros_like(self.low_t))
        self.armed &= ~(self.low_t >= self.LAND_DETECT_S)
        self.h_last = torch.where(z_valid, p[:, 2], self.h_last)
        blind = (self.mode == MODE_LAND) & ~z_valid
        self.blind_t = torch.where(blind, self.blind_t + self.dt, torch.zeros_like(self.blind_t))
        limit = self.h_last.clamp_min(0.0) / self.law.LAND_SPEED * self.BLIND_FACTOR + self.BLIND_MARGIN_S
        self.armed &= ~(self.blind_t >= limit)
        act = self.angle(roll, pitch, self.yaw_rate, thrust)
        off = ~self.armed
        act[off] = torch.tensor([-1.0, 0.0, 0.0, 0.0])
        return act


# ------------------------------------------------------------------------------- harness
def _eval_mod():
    from train import eval as ev
    ev.SENSING.setdefault("fw3", dict(gyro_filter=True, estimator=True, mahony=True, mahony_kf_ref=True))
    return ev


def _vehicles(spec, checks, ev):
    nom = ev.load_nominal()
    return {c: lite_vehicle(nom, spec, checks, c) for c in ("nominal", "pessimistic")}


def suite_a(ev, veh, spec, n, n_ext, seed, horizon_s=5.0):
    ev.load_nominal = lambda: copy.deepcopy(veh)
    env, groups, _ = ev.build(n, n_ext, seed, True, ("hover_1m", "tilt_60", "throw"), "fw3")
    ap = Autopilot(env, veh, spec)
    ap.law.p_sp[:] = torch.tensor(ev.TARGET)              # hold the eval target
    res = ev.rollout(env, ap, horizon_s=horizon_s)
    out = ev.summarize(res, groups)
    # rows whose randomized motors cannot lift the drone with margin are reported, not judged
    qp = env.quad.params
    fly = (qp.thrust_max.sum(-1) / (qp.mass * G)) >= MIN_TW
    for gr in groups:
        f = fly[gr.sl]
        out[gr.name]["unflyable"] = int((~f).sum())
        out[gr.name]["success_flyable"] = float(res["success"][gr.sl][f].float().mean()) if f.any() else float("nan")
    return out


def _hover_env(ev, veh, n, seed, randomized, pos=(0.0, 0.0, 1.5)):
    ev.load_nominal = lambda: copy.deepcopy(veh)
    env, groups, _ = ev.build(n, 1, seed, False, ("hover_1m",), "fw3")
    q = env.quad
    idx = torch.arange(q.n)
    if not randomized:
        nomp = ev.sample_params(q.n, veh, None, q.dt, use_nominal=True, max_latency_steps=q.max_delay)
        q.params.assign(idx, nomp)
        q._update_derived(idx)
    w = q.hover_motor_state(idx)
    q.set_state(idx, pos=torch.tensor(pos).expand(q.n, 3).clone(), vel=torch.zeros(q.n, 3), omega=torch.zeros(q.n, 3),
                quat=torch.tensor([[1.0, 0, 0, 0]]).expand(q.n, 4).clone(), motor=w, cmd=w)
    env.reinit_sensing(idx)
    env.cfg.oob_dist = 1e9
    return env


def _ground(env, state):
    """Simple ground contact: the first time z < 0 log the touchdown speed, then hold the
    drone on the floor (level, at rest) while the motors keep running."""
    q = env.quad
    below = q.pos[:, 2] <= 0.0
    first = below & ~state["landed"]
    state["td_speed"] = torch.where(first, -q.vel[:, 2], state["td_speed"])
    state["landed"] |= below
    idx = torch.nonzero(state["landed"]).flatten()
    if idx.numel():
        pos = q.pos[idx].clone()
        pos[:, 2] = 0.0
        z = torch.zeros(idx.numel(), 3)
        quat = torch.tensor([[1.0, 0, 0, 0]]).expand(idx.numel(), 4).clone()
        q.set_state(idx, pos=pos, vel=z, omega=z.clone(), quat=quat, motor=q.motor[idx])


def run_scenario(ev, veh, spec, n, seed, secs, randomized, events, pos=(0.0, 0.0, 1.5)):
    """Generic closed loop. events(k, t, env, ap) mutates sensors / commands."""
    env = _hover_env(ev, veh, n, seed, randomized, pos)
    env.quad.floor_z = -1e9
    ap = Autopilot(env, veh, spec)
    n = env.n
    st = {"landed": torch.zeros(n, dtype=torch.bool), "td_speed": torch.full((n,), float("nan"))}
    # on the floor the accelerometer feels the ground reaction (+g up), not just the thrust
    q, sample_accel = env.quad, env.quad.sample_accel

    # _ground stops a drone at the floor instantly, so the touchdown deceleration the real IMU feels
    # has to be fed to the accelerometer here: the remaining velocity change, spread over 1 kHz
    # samples at no more than the MPU-6050's 16 g range.
    st["impact"] = torch.zeros(n)
    st["td_t"] = torch.full((n,), float("nan"))
    dt_imu = 1.0 / env.fs
    a_sat = 16.0 * G - G

    def _accel_on_floor():
        a = sample_accel()
        L = st["landed"]
        if bool(L.any()):
            a[L] = a[L] - q.specific_force_b[L] + torch.tensor([0.0, 0.0, G])
            kick = (st["impact"] / dt_imu).clamp(max=a_sat)
            a[:, 2] += kick
            st["impact"] = st["impact"] - kick * dt_imu
        return a
    q.sample_accel = _accel_on_floor
    steps = int(secs / env.control_dt)
    log = {"pos": [], "vel": [], "mode": [], "armed": []}
    crashed = torch.zeros(n, dtype=torch.bool)
    disarm_t = torch.full((n,), float("nan"))
    for k in range(steps):
        t = k * env.control_dt
        events(k, t, env, ap)
        _, _, done, info = env.step(ap())
        was = st["landed"].clone()
        _ground(env, st)
        first = st["landed"] & ~was
        st["impact"] = torch.where(first, st["td_speed"].clamp_min(0.0), st["impact"])
        st["td_t"] = torch.where(first, torch.full_like(st["td_t"], t), st["td_t"])
        q = env.quad
        tilt = torch.acos(q.R[:, 2, 2].clamp(-1, 1))
        crashed |= (tilt > math.radians(60)) & ~st["landed"]
        newly = ~ap.armed & torch.isnan(disarm_t)
        disarm_t = torch.where(newly, torch.full_like(disarm_t, t), disarm_t)
        if k % 5 == 0:
            log["pos"].append(q.pos.clone()); log["vel"].append(q.vel.clone()); log["mode"].append(ap.mode.clone())
    lg = {k: torch.stack(v) for k, v in log.items() if k in ("pos", "vel", "mode")}
    # rows whose randomized motors cannot lift the drone with margin (T/W < 1.15, which the
    # pessimistic corner plus randomization produces) are reported, not judged
    qp = env.quad.params
    tw = qp.thrust_max.sum(-1) / (qp.mass * G)
    fly = tw >= MIN_TW
    lg["unflyable"] = int((~fly).sum())
    idx = torch.nonzero(fly).flatten()
    st = {k: v[idx] for k, v in st.items() if k != "impact"}
    lg = {k: (v[:, idx] if torch.is_tensor(v) else v) for k, v in lg.items()}
    return env, ap, st, crashed[idx], disarm_t[idx], lg


def suite_b(ev, veh, spec, seed):
    def ev_(k, t, env, ap):
        if k == int(1.0 / env.control_dt):
            ap.law.p_sp[:, 0] += 1.0
    env, ap, st, crashed, _, log = run_scenario(ev, veh, spec, 64, seed, 5.0, False, ev_)
    x = log["pos"][:, :, 0]
    dt5 = env.control_dt * 5
    t = torch.arange(x.shape[0]) * dt5
    x0 = x[int(0.99 / dt5)]
    rel = x - x0.unsqueeze(0)
    after = t >= 1.0
    peak = rel[after].amax(0)
    within = (rel - 1.0).abs() < 0.10
    settle = []
    for j in range(rel.shape[1]):
        bad = torch.nonzero(~within[:, j] & after).flatten()
        settle.append(float(t[bad[-1]] - 1.0 + dt5) if bad.numel() else 0.0)
    return {"unflyable": log["unflyable"], "overshoot_max": float((peak - 1.0).max()), "settle_s_max": max(settle), "crashed": int(crashed.sum())}


def suite_c(ev, veh, spec, n, seed):
    env, ap, st, crashed, _, log = run_scenario(ev, veh, spec, n, seed, 20.0, True, lambda *a: None)
    p = log["pos"][int(2.0 / (env.control_dt * 5)):]
    p0 = log["pos"][0]
    exy = (p[:, :, :2] - p0[:, :2]).norm(dim=-1)
    ez = (p[:, :, 2] - p0[:, 2]).abs()
    return {"unflyable": log["unflyable"], "rms_xy_m": float(exy.pow(2).mean().sqrt()), "max_xy_m": float(exy.max()),
            "rms_z_m": float(ez.pow(2).mean().sqrt()), "crashed": int(crashed.sum())}


def suite_d(ev, veh, spec, n, seed):
    def ev_(k, t, env, ap):
        ap.v_cmd[:, 0] = 0.5 if 1.0 <= t < 4.0 else 0.0
    env, ap, st, crashed, _, log = run_scenario(ev, veh, spec, n, seed, 6.0, True, ev_)
    dt5 = env.control_dt * 5
    vx = log["vel"][int(2.5 / dt5):int(4.0 / dt5), :, 0]
    return {"unflyable": log["unflyable"], "speed_err_max": float((vx - 0.5).abs().max()), "crashed": int(crashed.sum())}


def suite_e(ev, veh, spec, n, seed):
    def ev_(k, t, env, ap):
        if k == int(2.0 / env.control_dt):
            env.kf.min_quality = 2.0                     # every flow sample now fails the quality gate
    env, ap, st, crashed, _, log = run_scenario(ev, veh, spec, n, seed, 7.0, True, ev_)
    dt5 = env.control_dt * 5
    z = log["pos"][int(2.0 / dt5):, :, 2]
    z0 = log["pos"][int(2.0 / dt5) - 1, :, 2]
    drift = (log["pos"][-1, :, :2] - log["pos"][int(2.0 / dt5), :, :2]).norm(dim=-1)
    return {"unflyable": log["unflyable"], "height_err_max": float((z - z0).abs().max()), "drift_xy_5s_median_m": float(drift.median()),
            "drift_xy_5s_max_m": float(drift.max()), "mode_alt_frac": float((log["mode"][-1] == MODE_ALT).float().mean()),
            "crashed": int(crashed.sum())}


def suite_f(ev, veh, spec, n, seed):
    def ev_(k, t, env, ap):
        if k == int(2.0 / env.control_dt):
            env.kf.max_range = 0.0                       # every range sample now invalid
    env, ap, st, crashed, disarm_t, log = run_scenario(ev, veh, spec, n, seed, 12.0, True, ev_)
    return {"unflyable": log["unflyable"], "touchdown_speed_max": float(st["td_speed"].nan_to_num(9.9).max()),
            "touchdown_speed_median": float(st["td_speed"].nan_to_num(9.9).median()),
            "landed": float(st["landed"].float().mean()), "disarm_s_max": float(disarm_t.nan_to_num(99.0).max()),
            "disarm_after_touchdown_max_s": float((disarm_t - st["td_t"]).nan_to_num(99.0).max()),
            "crashed": int(crashed.sum())}


def suite_g(ev, veh, spec, n, seed):
    def ev_(k, t, env, ap):
        if k == int(1.0 / env.control_dt):
            ap.request_land(torch.arange(env.n))
    env, ap, st, crashed, disarm_t, log = run_scenario(ev, veh, spec, n, seed, 9.0, True, ev_)
    return {"unflyable": log["unflyable"], "touchdown_speed_max": float(st["td_speed"].nan_to_num(9.9).max()),
            "landed": float(st["landed"].float().mean()), "disarm_s_max": float(disarm_t.nan_to_num(99.0).max()),
            "disarm_after_touchdown_max_s": float((disarm_t - st["td_t"]).nan_to_num(99.0).max()),
            "crashed": int(crashed.sum())}


# Eval-suite gate on the nominal / randomized groups. The pessimistic corner (T/W 1.27) gets 8 s to
# recover from a 60 deg tilt or a throw instead of 5 s; the single-parameter extreme groups are reported only.
GATE_A = {"nominal": 0.95, "pessimistic": 0.90}
HORIZON_A = {"nominal": 5.0, "pessimistic": 8.0}


def main(argv=None):
    ap_ = argparse.ArgumentParser()
    ap_.add_argument("--n", type=int, default=256)
    ap_.add_argument("--n-ext", type=int, default=32)
    ap_.add_argument("--seed", type=int, default=11)
    ap_.add_argument("--only", default="ABCDEFG")
    a = ap_.parse_args(argv)
    torch.set_num_threads(max(1, os.cpu_count() or 1))
    from .frame import load_spec
    spec = load_spec()
    with open(os.path.join(OUT, "checks.json")) as f:
        checks = json.load(f)
    ev = _eval_mod()
    vehs = _vehicles(spec, checks, ev)
    out, fails = {}, []
    for corner, veh in vehs.items():
        r = out.setdefault(corner, {})
        if "A" in a.only:
            r["A"] = suite_a(ev, veh, spec, a.n, a.n_ext, a.seed, HORIZON_A[corner])
            worst = min(((v["success"], k) for k, v in r["A"].items()))
            print(f"[{corner}] A worst {worst[1]} {worst[0]*100:.1f}%  " +
                  ", ".join(f"{k} {v['success_flyable']*100:.0f}% (unflyable {v['unflyable']})"
                            for k, v in r["A"].items() if "/" in k and "=" not in k))
            for k, v in r["A"].items():
                if "=" not in k and not v["success_flyable"] >= GATE_A[corner]:
                    fails.append(f"{corner} A {k}: {v['success_flyable']*100:.1f}%")
        n = min(a.n, 128)
        for key, fn, args in (("B", suite_b, ()), ("C", suite_c, (n,)), ("D", suite_d, (n,)), ("E", suite_e, (n,)),
                              ("F", suite_f, (n,)), ("G", suite_g, (n,))):
            if key in a.only:
                r[key] = fn(ev, veh, spec, *args, a.seed)
                print(f"[{corner}] {key} {r[key]}")
        b, c, d, e, f, g = (r.get(k) for k in "BCDEFG")
        chk = [(b, lambda x: x["overshoot_max"] < 0.20 and x["settle_s_max"] <= 3.0 and not x["crashed"], "B step"),
               (c, lambda x: x["rms_xy_m"] < 0.10 and x["rms_z_m"] < 0.05 and not x["crashed"], "C hold"),
               (d, lambda x: x["speed_err_max"] < 0.10 and not x["crashed"], "D velocity"),
               (e, lambda x: x["height_err_max"] < 0.20 and not x["crashed"], "E flow loss"),
               # F: a blind descent's true speed depends on the accelerometer bias, so touchdown can come late;
               # judge the landing detector (disarm soon after touchdown) and the blind-land timeout bound
               (f, lambda x: x["touchdown_speed_max"] < 1.0 and x["disarm_after_touchdown_max_s"] <= 2.0
                and x["disarm_s_max"] <= 12.0 and not x["crashed"], "F range loss"),
               (g, lambda x: x["touchdown_speed_max"] < 0.6 and x["disarm_s_max"] <= 8.0
                and x["disarm_after_touchdown_max_s"] <= 2.0 and not x["crashed"], "G land")]
        for res, ok, name in chk:
            if res is not None and not ok(res):
                fails.append(f"{corner} {name}: {res}")
    out["failures"] = fails
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "autonomy.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print("FAIL" if fails else "PASS", *fails, sep="\n  ")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

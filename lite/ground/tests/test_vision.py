"""Detector accuracy on synthetic frames with known geometry, and TagController behaviour."""
import math

import numpy as np

from lite.ground import link
from lite.ground.control import ControlConfig, TagController
from lite.ground.vision import Camera, TagDetector
from lite.vision_sim import CAM_OFFSET_B, TagWorld, _background, render

TAG = TagWorld((2.0, 0.0, 1.0), (-1.0, 0.0, 0.0), 7)


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def test_pose_accuracy_over_range():
    cam, rng = Camera(), np.random.default_rng(2)
    det, bg = TagDetector(cam), _background(rng, cam)
    worst = {}
    for x in (1.3, 0.8, 0.0, -0.8, -1.5):            # 0.65 .. 3.5 m from the tag
        for y in (-0.4, 0.0, 0.4):
            for yaw_deg in (-15, 0, 15):
                pos = np.array([x, y, 1.05])
                R = Rz(math.radians(yaw_deg))
                if abs(math.atan2(-y, 2.0 - x) - math.radians(yaw_deg)) > math.radians(26):
                    continue                        # outside the +-30 deg view
                poses = det.detect(render(cam, TAG, pos, R, bg, rng))
                assert len(poses) == 1 and poses[0].tag_id == 7, (x, y, yaw_deg)
                cam_w = pos + R @ CAM_OFFSET_B
                truth = R.T @ (TAG.c - cam_w)       # tag in the heading frame (level drone)
                got = np.array(poses[0].body())
                d = float(np.linalg.norm(truth))
                worst[round(d, 1)] = max(worst.get(round(d, 1), 0), float(np.linalg.norm(got - truth)) / d)
    # position error < 3 % of the range everywhere out to 3.5 m (~1 % at 1 m)
    assert max(worst.values()) < 0.03, worst


def _pose_at(pos, yaw=0.0):
    cam, rng = Camera(), np.random.default_rng(3)
    return TagDetector(cam).detect(render(cam, TAG, np.array(pos), Rz(yaw), _background(rng, cam), rng))


def test_controller_follow_and_lost_and_search():
    c = TagController(ControlConfig(task="follow", dist=0.8))
    poses = _pose_at((0.4, 0.3, 1.0))
    cmd = c.update(0.0, poses)
    f, l, u, w = cmd.velocity_si()
    # forward, right (the face normal from one 1.6 m view is a few degrees off, so the lateral part is
    # under-estimated until closer), turn right
    assert cmd.op == link.OP_VEL and f > 0.3 and l < -0.05 and w < 0
    t = 0.0
    for _ in range(5):                                 # tag lost: < 0.4 s keeps flying (dead-reckoned)
        t += 1 / 15
        cmd = c.update(t, [])
    assert cmd.op == link.OP_VEL
    for _ in range(10):
        t += 1 / 15
        cmd = c.update(t, [])
    assert cmd.op == link.OP_HOLD and "lost" in c.status
    while t < 4.0:
        t += 1 / 15
        cmd = c.update(t, [])
    _, _, _, w = cmd.velocity_si()
    assert c.status == "searching" and w < 0           # turns the way the tag went (right)


def test_controller_lands_when_settled():
    c = TagController(ControlConfig(task="land", land_dist=0.5))
    # exactly at the land point, facing the tag
    poses = _pose_at((2.0 - 0.5 - CAM_OFFSET_B[0] * 0, 0.0, 0.98))
    t, op = 0.0, None
    for _ in range(40):
        t += 1 / 15
        op = c.update(t, poses).op
        if op == link.OP_LAND:
            break
    assert op == link.OP_LAND and 0.9 <= t <= 1.5
    assert c.update(t + 1, []).op == link.OP_LAND      # latched

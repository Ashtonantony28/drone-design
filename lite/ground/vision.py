"""AprilTag (36h11) detection and pose from the XIAO camera stream (OpenCV only).

Frames: camera (OpenCV): x right, y down, z forward. The camera looks along the drone's nose
(body x), so body forward = cam z, body left = -cam x, body up = -cam y (CAM_PITCH_DEG tilts it).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

DICT = cv2.aruco.DICT_APRILTAG_36h11
TAG_SIZE_M = 0.15            # black square edge of the printed tag: fits A4 (lite/ground/make_tags.py)
CAM_PITCH_DEG = 0.0          # camera tilt, + = looking down (the arch holds it level)


@dataclass
class Camera:
    """Pinhole model. Defaults: OV2640 stock lens at VGA, ~60 deg horizontal FOV. Replace with a
    calibration (python -m lite.ground.calibrate) for best range accuracy."""
    width: int = 640
    height: int = 480
    fx: float = 554.0
    fy: float = 554.0
    cx: float = 320.0
    cy: float = 240.0
    dist: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float64)

    @property
    def D(self) -> np.ndarray:
        return np.array(self.dist, dtype=np.float64)

    @staticmethod
    def load(path: str) -> "Camera":
        import yaml
        d = yaml.safe_load(open(path))
        return Camera(**{k: (tuple(v) if k == "dist" else v) for k, v in d.items()})


@dataclass
class TagPose:
    tag_id: int
    t_cam: np.ndarray        # tag centre in the camera frame (m)
    normal_cam: np.ndarray   # tag normal (out of the printed face) in the camera frame
    corners: np.ndarray      # 4x2 pixels
    err_px: float            # reprojection error

    def body(self, pitch_deg: float = CAM_PITCH_DEG):
        """Tag centre as (forward, left, up) in the drone's heading frame (camera at the nose)."""
        x, y, z = self.t_cam
        p = math.radians(pitch_deg)
        fwd = z * math.cos(p) - y * math.sin(p)
        up = -(y * math.cos(p) + z * math.sin(p))
        return float(fwd), float(-x), float(up)

    def bearing(self) -> float:
        """Horizontal angle to the tag, + = to the left (rad)."""
        f, l, _ = self.body()
        return math.atan2(l, f)

    def normal_body(self, pitch_deg: float = CAM_PITCH_DEG):
        """Tag face normal (out of the print, towards the viewer) as (forward, left, up)."""
        x, y, z = self.normal_cam
        p = math.radians(pitch_deg)
        return float(z * math.cos(p) - y * math.sin(p)), float(-x), float(-(y * math.cos(p) + z * math.sin(p)))

    def standoff_point(self, dist: float):
        """The point `dist` metres out from the tag along its face normal, horizontal only, at the
        tag's height: (forward, left, up) in the heading frame. Flying there faces the tag squarely."""
        f, l, u = self.body()
        nf, nl, _ = self.normal_body()
        nh = math.hypot(nf, nl)
        if nh < 0.3:                 # tag seen almost edge-on from above/below: keep straight back
            nf, nl, nh = -1.0, 0.0, 1.0
        return f + dist * nf / nh, l + dist * nl / nh, u


class TagDetector:
    def __init__(self, cam: Camera, tag_size: float = TAG_SIZE_M):
        self.cam, self.s = cam, tag_size
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT), params)
        h = tag_size / 2
        self.obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    def detect(self, gray: np.ndarray) -> list[TagPose]:
        corners, ids, _ = self.det.detectMarkers(gray)
        out = []
        if ids is None:
            return out
        for c, i in zip(corners, ids.flatten()):
            img = c.reshape(4, 2).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(self.obj, img, self.cam.K, self.cam.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(self.obj, rvec, tvec, self.cam.K, self.cam.D)
            err = float(np.abs(proj.reshape(4, 2) - img).max())
            R, _ = cv2.Rodrigues(rvec)
            normal = R[:, 2]         # marker z: out of the printed face, towards the camera
            out.append(TagPose(int(i), tvec.flatten(), normal, img, err))
        return out

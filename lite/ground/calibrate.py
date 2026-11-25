"""Camera calibration from the drone's stream with a printed chessboard (9 x 6 inner corners).

    python -m lite.ground.calibrate --square 0.025        (square size in metres, as printed)

Hold the board in front of the camera at different distances and angles; a frame is taken
each time the board is found (max 2 per second) until 25 are collected. Writes
lite/ground/camera.yaml; pass it to the station with --camera lite/ground/camera.yaml.
"""
import argparse
import os
import time

import cv2
import numpy as np
import yaml

from .station import LatestFrame


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--drone", default="192.168.4.1")
    ap.add_argument("--square", type=float, required=True)
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--n", type=int, default=25)
    a = ap.parse_args(argv)
    pattern = (a.cols, a.rows)
    obj = np.zeros((a.cols * a.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
    stream = LatestFrame(f"http://{a.drone}:81/stream")
    objs, imgs, last, size = [], [], 0.0, None
    while len(imgs) < a.n:
        f, t, _ = stream.get()
        if f is None or time.monotonic() - last < 0.5:
            time.sleep(0.02)
            continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        size = g.shape[::-1]
        ok, c = cv2.findChessboardCorners(g, pattern)
        if ok:
            c = cv2.cornerSubPix(g, c, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
            objs.append(obj)
            imgs.append(c)
            last = time.monotonic()
            print(f"frame {len(imgs)}/{a.n}")
    rms, K, D, _, _ = cv2.calibrateCamera(objs, imgs, size, None, None)
    out = {"width": size[0], "height": size[1], "fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]),
           "cy": float(K[1, 2]), "dist": [float(x) for x in D.flatten()[:5]]}
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera.yaml")
    yaml.safe_dump(out, open(path, "w"))
    print(f"RMS reprojection {rms:.2f} px (good: < 0.5) -> {path}")
    stream.ok = False


if __name__ == "__main__":
    main()

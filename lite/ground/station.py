"""NF-3D Lite ground station: watch the drone's camera, find AprilTags, fly tag-relative.

    pip install opencv-python numpy pyyaml          (once, on the laptop)
    join Wi-Fi "NF3D-Lite" (password nf3d-lite), then:
    python -m lite.ground.station --task follow     (or --task land, --tag 3, --dist 1.0)

Safety model: the laptop can only steer. The pilot arms, takes off and presses A (position hold)
with the Xbox controller, then Y (AUTO). The station starts PAUSED (it sends HOLD), so AUTO
engages without moving; press G to let the tag controller fly. Space pauses again, L lands,
Q quits (sends HOLD; the flight board falls back to position hold 2 s after the laptop goes
quiet). B on the controller always kills the motors; X takes manual control back.

Keys (video window): G go, Space pause/hold, L land, Q / Esc quit.
"""
from __future__ import annotations

import argparse
import socket
import threading
import time

import cv2
import numpy as np

from . import link
from .control import ControlConfig, TagController
from .vision import Camera, TagDetector

UDP_PORT = 14560


class LatestFrame:
    """Reads the MJPEG stream on a thread and keeps only the newest frame (latency over smoothness)."""

    def __init__(self, url: str):
        self.cap = cv2.VideoCapture(url)
        self.frame, self.t, self.n = None, 0.0, 0
        self.ok = self.cap.isOpened()
        self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.ok:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._lock:
                self.frame, self.t, self.n = f, time.monotonic(), self.n + 1

    def get(self):
        with self._lock:
            return self.frame, self.t, self.n


def draw(img, poses, ctl: TagController, telem, paused: bool, fps: float):
    for p in poses:
        c = p.corners.astype(int)
        cv2.polylines(img, [c], True, (0, 255, 0), 2)
        f, l, u = p.body()
        cv2.putText(img, f"id {p.tag_id}  {f:.2f} m", tuple(c[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    lines = [("PAUSED (G to go)" if paused else ctl.status), f"{fps:.0f} fps"]
    if telem is not None:
        lines.append(f"{telem.mode_name}  {link.STATES[telem.state] if telem.state < 3 else '?'}  "
                     f"{telem.vbat_mv / 1000:.2f} V  h {telem.pos_cm[2] / 100:.2f} m")
    else:
        lines.append("no telemetry: is the flight board wired to the XIAO?")
    for i, s in enumerate(lines):
        cv2.putText(img, s, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return img


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drone", default="192.168.4.1")
    ap.add_argument("--task", choices=("follow", "land"), default="follow")
    ap.add_argument("--tag", type=int, default=None, help="tag id to use (default: nearest)")
    ap.add_argument("--dist", type=float, default=0.8, help="follow stand-off (m)")
    ap.add_argument("--camera", default=None, help="camera.yaml from python -m lite.ground.calibrate")
    ap.add_argument("--no-window", action="store_true", help="print status instead of showing video")
    ap.add_argument("--record", default=None, help="save the annotated video (.mp4)")
    ap.add_argument("--stream-port", type=int, default=81)
    ap.add_argument("--udp-port", type=int, default=UDP_PORT)
    ap.add_argument("--seconds", type=float, default=0.0, help="quit after this long (tests)")
    a = ap.parse_args(argv)

    cam = Camera.load(a.camera) if a.camera else Camera()
    det = TagDetector(cam)
    ctl = TagController(ControlConfig(task=a.task, target_id=a.tag, dist=a.dist))
    url = f"http://{a.drone}:{a.stream_port}/stream"
    stream = LatestFrame(url)
    if not stream.ok:
        raise SystemExit(f"cannot open {url}: join the NF3D-Lite Wi-Fi first")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    dest = (a.drone, a.udp_port)
    t_end = time.monotonic() + a.seconds if a.seconds > 0 else float("inf")
    parser = link.Parser()
    telem, paused, last_n, seq = None, True, -1, 0
    fps, t_fps = 0.0, time.monotonic()
    writer = None
    try:
        while time.monotonic() < t_end:
            # telemetry / sensor frames from the drone
            try:
                while True:
                    data, _ = sock.recvfrom(512)
                    for fr in parser.feed(data):
                        if isinstance(fr, link.Telem):
                            telem = fr
            except BlockingIOError:
                pass
            frame, t, n = stream.get()
            if frame is None or n == last_n:
                # no new frame: keep the link alive at ~10 Hz with the last decision
                time.sleep(0.01)
                continue
            last_n = n
            now = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(now - t_fps, 1e-3)
            t_fps = now
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            poses = det.detect(gray)
            cmd = ctl.update(t, poses)
            if paused:
                seq = (seq + 1) & 0xFFFF
                cmd = link.Cmd(link.OP_HOLD, seq=seq)
            sock.sendto(cmd.pack(), dest)
            vis = draw(frame, poses, ctl, telem, paused, fps)
            if a.record:
                if writer is None:
                    writer = cv2.VideoWriter(a.record, cv2.VideoWriter_fourcc(*"mp4v"), 15, (vis.shape[1], vis.shape[0]))
                writer.write(vis)
            if a.no_window:
                if a.seconds > 0 and paused:
                    paused = False      # test mode: fly straight away
                if n % 15 == 0:
                    print(("PAUSED " if paused else "") + ctl.status, telem.mode_name if telem else "-", flush=True)
                continue
            cv2.imshow("NF-3D Lite", vis)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord(" "):
                paused = True
            elif k == ord("g"):
                paused = False
                ctl = TagController(ControlConfig(task=a.task, target_id=a.tag, dist=a.dist))
            elif k == ord("l"):
                for _ in range(5):
                    seq = (seq + 1) & 0xFFFF
                    sock.sendto(link.Cmd(link.OP_LAND, seq=seq).pack(), dest)
                paused = True
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            sock.sendto(link.Cmd(link.OP_HOLD, seq=(seq + 1) & 0xFFFF).pack(), dest)
        if writer is not None:
            writer.release()
        stream.ok = False


if __name__ == "__main__":
    main()

"""End-to-end run of the ground station app against a fake drone on localhost: an MJPEG HTTP
stream of synthetic camera frames (the drone 0.5 m left of and 1.6 m from a tag) and a UDP
endpoint standing in for the XIAO. The station must open the stream, detect the tag, and send
CMD frames that steer towards the stand-off point (forward + right) and yaw towards the tag."""
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from lite.ground import link, station
from lite.ground.vision import Camera
from lite.vision_sim import TagWorld, _background, render


def _frame():
    cam = Camera()
    rng = np.random.default_rng(1)
    tag = TagWorld((2.0, 0.0, 1.0), (-1.0, 0.0, 0.0), 0)
    gray = render(cam, tag, np.array([0.4, 0.5, 1.0]), np.eye(3), _background(rng, cam), rng)
    ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
    return jpg.tobytes()


class _Mjpeg(BaseHTTPRequestHandler):
    jpg = b""

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace;boundary=nf3dframe")
        self.end_headers()
        try:
            for _ in range(200):
                self.wfile.write(b"\r\n--nf3dframe\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(self.jpg))
                self.wfile.write(self.jpg)
                time.sleep(1 / 15)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *a):
        pass


def test_station_end_to_end():
    _Mjpeg.jpg = _frame()
    http = ThreadingHTTPServer(("127.0.0.1", 0), _Mjpeg)
    threading.Thread(target=http.serve_forever, daemon=True).start()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("127.0.0.1", 0))
    udp.settimeout(0.2)
    got = []

    def rx():
        p = link.Parser()
        t_end = time.time() + 8
        while time.time() < t_end:
            try:
                data, addr = udp.recvfrom(512)
            except socket.timeout:
                continue
            got.extend(p.feed(data))
            udp.sendto(link.Telem(mode=4, state=2, vbat_mv=3900).pack(), addr)   # the XIAO relays TELEM back
    th = threading.Thread(target=rx, daemon=True)
    th.start()
    station.main(["--drone", "127.0.0.1", "--stream-port", str(http.server_address[1]),
                  "--udp-port", str(udp.getsockname()[1]), "--no-window", "--seconds", "4"])
    th.join()
    http.shutdown()
    vel = [c for c in got if isinstance(c, link.Cmd) and c.op == link.OP_VEL]
    assert len(vel) >= 20, f"only {len(vel)} velocity commands"
    f, l, u, w = vel[-1].velocity_si()
    # stand-off point 0.8 m in front of the tag: 0.8 m ahead, 0.5 m to the right of the drone
    assert f > 0.2 and l < -0.2, (f, l)
    assert w < 0.0                       # tag is to the right: turn clockwise
    assert abs(u) < 0.1

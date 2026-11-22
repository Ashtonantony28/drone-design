"""Printable AprilTags (36h11) for the vision behaviours: one A4 page per tag, 150 mm black
square (lite/ground/vision.py TAG_SIZE_M), white margin, id label.

    python -m lite.ground.make_tags 0 1 2        -> lite/out/tags/tag_0.png ... (300 dpi)

Print at 100 % ("actual size", not "fit to page") and check the black square measures 150 mm;
if it does not, pass the measured size to the station (--tag-size is TAG_SIZE_M in vision.py).
Tape it flat: a curled tag gives a wrong distance.
"""
import os
import sys

import cv2
import numpy as np

from .vision import DICT, TAG_SIZE_M

DPI = 300
A4_MM = (210, 297)


def page(tag_id: int) -> np.ndarray:
    px_mm = DPI / 25.4
    W, H = int(A4_MM[0] * px_mm), int(A4_MM[1] * px_mm)
    img = np.full((H, W), 255, np.uint8)
    side = int(round(TAG_SIZE_M * 1000 * px_mm))
    mk = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(DICT), tag_id, side)
    x0, y0 = (W - side) // 2, int(40 * px_mm)
    img[y0:y0 + side, x0:x0 + side] = mk
    cv2.putText(img, f"NF-3D Lite tag {tag_id}  (36h11, {TAG_SIZE_M * 1000:.0f} mm)", (x0, y0 + side + int(15 * px_mm)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, 0, 3)
    return img


def main(argv=None):
    ids = [int(a) for a in (argv if argv is not None else sys.argv[1:])] or [0]
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out", "tags")
    os.makedirs(out, exist_ok=True)
    for i in ids:
        p = os.path.join(out, f"tag_{i}.png")
        cv2.imwrite(p, page(i), [cv2.IMWRITE_PNG_COMPRESSION, 9])
        print("wrote", p)


if __name__ == "__main__":
    main()

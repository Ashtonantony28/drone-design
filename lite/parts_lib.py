"""Shape helpers for the NF-3D Lite detailed model (lite/detail.py).

Bought parts are modelled from their datasheet outlines with the features that make them
read as the real thing: rounded and chamfered mould edges, gull-wing and through-hole
leads, connector openings, pin headers with solder fillets. Every helper returns solids in
the body frame (x forward, y left, z up, mm).
"""
from __future__ import annotations

import math

from build123d import (Align, Axis, Box, Circle, Cone, Cylinder, FilletPolyline, Part, Plane, Polyline, Pos, Rot,
                       chamfer, fillet, sweep)

C = (Align.CENTER, Align.CENTER, Align.MIN)
CC = (Align.CENTER, Align.CENTER, Align.CENTER)


def _try(fn, fallback):
    try:
        r = fn()
        return r if r is not None and r.is_valid and r.volume > 0 else fallback
    except Exception:
        return fallback


def rbox(x0, x1, y0, y1, z0, h, rv=0.0, rt=0.0, rb=0.0):
    """Axis-aligned box from x0..x1, y0..y1, z0..z0+h with its vertical edges rounded (rv),
    and its top (rt) / bottom (rb) edges rounded. Radii are clamped to what fits."""
    lx, ly = x1 - x0, y1 - y0
    b = Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(lx, ly, h, align=C)
    lim = min(lx, ly) / 2 - 1e-3
    if rv > 0:
        r = min(rv, lim)
        b = _try(lambda: fillet(b.edges().filter_by(Axis.Z), r), b)
    for rr, pick in ((rt, -1), (rb, 0)):
        if rr > 0:
            r = min(rr, lim, h / 2 - 1e-3)
            if r > 0.01:
                b = _try(lambda: fillet(b.edges().group_by(Axis.Z)[pick], r), b)
    return b


def obox(cx, cy, z0, L, W, h, ang=0.0, rv=0.0, rt=0.0, rb=0.0):
    """rbox centred at (cx, cy) with its length L along angle ang (deg)."""
    b = rbox(-L / 2, L / 2, -W / 2, W / 2, 0, h, rv, rt, rb)
    return Pos(cx, cy, z0) * Rot(0, 0, ang) * b


def ycyl(x, y, z, r, L):
    """Cylinder along +y starting at y."""
    return Pos(x, y, z) * Rot(-90, 0, 0) * Cylinder(r, L, align=C)


def xcyl(x, y, z, r, L):
    """Cylinder along +x starting at x."""
    return Pos(x, y, z) * Rot(0, 90, 0) * Cylinder(r, L, align=C)


def wire(points, r=0.35, bend=1.0):
    """A round wire swept along a 3-D polyline with rounded bends."""
    pts = []
    for p in points:                                     # drop repeated points
        q = tuple(map(float, p))
        if not pts or math.dist(q, pts[-1]) > 1e-6:
            pts.append(q)
    path = FilletPolyline(*pts, radius=bend) if len(pts) > 2 else Polyline(*pts)
    d = [pts[1][i] - pts[0][i] for i in range(3)]
    prof = Plane(origin=pts[0], z_dir=d) * Circle(r)
    return sweep(prof, path=path)


def square_pin(x, y, z0, z1, s=0.64, tip=0.15):
    """A 0.64 mm square header pin from z0 to z1 with chamfered ends."""
    p = Pos(x, y, z0) * Box(s, s, z1 - z0, align=C)
    return _try(lambda: chamfer(p.edges().group_by(Axis.Z)[-1] + p.edges().group_by(Axis.Z)[0], tip), p)


def solder_fillet(x, y, z, r_pad, r_pin, h, up=True, square=None):
    """Cone of solder on a pad round a pin or wire (pin removed, so it only touches it)."""
    if up:
        c = Pos(x, y, z) * Cone(r_pad, r_pin, h, align=C)
    else:
        c = Pos(x, y, z - h) * Cone(r_pin, r_pad, h, align=C)
    hole = (Pos(x, y, z - h - 1) * Box(square, square, h + 2, align=C) if square
            else Pos(x, y, z - h - 1) * Cylinder(r_pin, h + 2, align=C))
    return c - hole


def gullwing_leads(body_cx, body_cy, ang, half_w, feet, z_board, h_body, standoff, down=True, t=0.13, w=0.42):
    """Gull-wing leads from the long sides of a moulded body to its pads, as one solid.

    feet: list of (x, y, foot_len) in world XY; each foot lies flat on its pad surface at
    z_board. The body spans standoff..standoff+h_body away from the board face (down=True:
    hanging under a bottom-side face). A lead leaves the body side at mid-height, drops to
    the pad and runs out as a flat foot."""
    s = -1.0 if down else 1.0
    ca, sa = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    zsh = s * (standoff + h_body * 0.45)                  # shoulder centre, relative to the pad face
    out = None
    for fx, fy, flen in feet:
        dx, dy = fx - body_cx, fy - body_cy
        lx, ly = dx * ca + dy * sa, -dx * sa + dy * ca    # foot centre in the body frame
        sg = 1.0 if ly >= 0 else -1.0
        y_side = sg * half_w
        y_tip = ly + sg * flen / 2
        y_bend = ly - sg * flen / 2 + sg * 0.05            # inner end of the foot
        parts = [
            # shoulder: out of the body side to the bend, at mid-body height
            Pos(lx, (y_side + y_bend) / 2, zsh - t / 2) * Box(w, abs(y_bend - y_side), t, align=C),
            # drop from the shoulder to the pad
            # (ends at the pad face: down -> zsh - t/2 .. 0, up -> 0 .. zsh + t/2)
            (Pos(lx, y_bend, zsh - t / 2) * Box(w, t, -zsh + t / 2, align=C) if s < 0
             else Pos(lx, y_bend, 0.0) * Box(w, t, zsh + t / 2, align=C)),
            # foot on the pad
            Pos(lx, (y_bend + y_tip) / 2, 0.0 if s > 0 else -t) * Box(w, abs(y_tip - y_bend), t, align=C),
        ]
        lead = parts[0] + parts[1] + parts[2]
        lead = Pos(body_cx, body_cy, z_board) * Rot(0, 0, ang) * lead
        out = lead if out is None else out + lead
    return out

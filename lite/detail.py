"""NF-3D Lite detailed visual model (for the browser viewer, renders and the geometry audit).

The checks in lite/checks.py run on simple envelopes (a cylinder per motor, a box per
module). This file builds what the parts actually look like, from the same spec numbers
and the board's real layout (lite/ecad/layout/build/board.json), and it is assembled the
way the drone is built: every part rests on, plugs into, is soldered to, screwed to,
taped to or tied to the part that holds it. lite/audit.py proves that (no overlaps, no
floating parts) on the exact solids.

  motors     can with vent slots, end cap on the tube ledge, bearing boss, shaft through
             the prop hub, zip tie in the clamp groove, two leads routed out of a vent
             window, along the arm and soldered into the motor pads
  props      hub + two lofted, twisted blades (CW and CCW are mirror images)
  board      FR4 core, milled copper pour both sides with isolation round every pad, tinned
             pads, plated drills; bottom-side SMD parts with real bodies, terminations and
             gull-wing leads; through-hole parts with pins and solder fillets; M2 screws
  modules    ESP32 MiniKit on female headers (male pins in the socket cavities), GY-521 on
             its pin header, TP4056 module on three pins, battery lead and PH2.0 plug
  autonomy   PMW3901 / VL53L1X breakouts on foam tape on the tail-tab rims, XIAO ESP32-S3
             Sense stack (board-to-board connector, OV2640 with its lens through the arch)
             held to the arch by a zip tie
  battery    1S pouch on a foam pad, held by a strap through the plate slots

    python -m lite.detail          part count + fit check: every detailed part stays inside
the envelope the checks used for it, so the clearance numbers in checks.json still hold.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass

from build123d import (Align, Axis, Box, Cylinder, Ellipse, Part, Plane, Polygon, Pos, Rot, Sphere, extrude, fillet,
                       loft, revolve)

from .frame import C, build_assembly, dims, frame, guard, load_spec, motor_xy, prop_disk
from .parts_lib import (gullwing_leads, obox, rbox, solder_fillet, square_pin, wire, xcyl)

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD_JSON = os.path.join(HERE, "ecad", "layout", "build", "board.json")
CW = {0, 2}  # BUILD.md motor direction table
CU, SOL = 0.035, 0.05   # copper and tin thickness (mm)

# colours (sRGB hex) and finish
PETG, PETG_GUARD = 0xE67E22, 0x5D6D7E
STEEL, SILVER, BLACK, DARK = 0xB8BCC2, 0xD0D3D6, 0x1C1C1E, 0x2B2F33
COPPER, FR4, SOLDER, TIN = 0xC98A4B, 0xD9C68E, 0xC9CCCF, 0xBFC3C7
BLUE_PCB, BLACK_PCB, PURPLE_PCB = 0x1F5AA6, 0x202326, 0x6C3483
PROP_CW, PROP_CCW = 0xBFC5CC, 0x34495E
RED, WIRE_BLK, CAP_TAN, LIPO = 0xC0392B, 0x222222, 0x9C7A4F, 0xC9D1D8
NYLON, FOAM, STRAP = 0xF2EFE6, 0x2C3E50, 0x1B2631
GOLD = 0xC9A227


@dataclass
class DPart:
    name: str        # shown on hover
    group: str       # viewer toggle group
    part: Part
    color: int
    metal: float = 0.1
    rough: float = 0.6
    owner: str | None = None   # envelope component (lite.frame.build_assembly) it must stay inside
    opacity: float = 1.0


def _b2f(x, y):
    """Board layout coordinates (KiCad, y down) to body FLU, as lite/ecad places the board."""
    return x, -y


def _union(parts):
    out = None
    for p in parts:
        if p is not None:
            out = p if out is None else out + p
    return out


# ------------------------------------------------------------------ layout access
def _layout():
    with open(BOARD_JSON) as f:
        return json.load(f)


def _pads(fp):
    """Pads of a footprint in FLU with their size oriented by the absolute pad angle."""
    out = []
    for p in fp["pads"]:
        if p.get("npth"):
            continue
        x, y = _b2f(*p["at"])
        w, h = p["size"]
        if round(p.get("angle", 0.0)) % 180 == 90:
            w, h = h, w
        out.append({"x": x, "y": y, "w": w, "h": h, "shape": p.get("shape", "rect"), "drill": p.get("drill") or 0.0})
    return out


def _pad_solid(p, z, th, grow=0.0):
    w, h = p["w"] + 2 * grow, p["h"] + 2 * grow
    if p["shape"] == "circle":
        return Pos(p["x"], p["y"], z) * Cylinder(max(w, h) / 2, th, align=C)
    rv = min(w, h) / 2 - 1e-3 if p["shape"] == "oval" else (0.25 if grow else 0.0)
    return rbox(p["x"] - w / 2, p["x"] + w / 2, p["y"] - h / 2, p["y"] + h / 2, z, th, rv=rv)


# ------------------------------------------------------------------ motors and props
def motor_detail(spec, i, x, y):
    g, d = spec["geometry_mm"], dims(spec)
    r = spec["parts"]["motor"]["envelope_mm"]["diameter"] / 2 - 0.05  # as lite.frame.motor()
    z0, z1 = g["motor_z0"], d["motor_z1"]
    out = []
    can = Pos(x, y, z0 + 1.5) * Cylinder(r, z1 - z0 - 2.3, align=C)
    try:
        can = fillet(can.edges().group_by(Axis.Z)[-1], 0.5)
    except Exception:
        pass
    for k in range(4):  # vent slots near the top of the can
        a = 45 + 90 * k
        can -= Pos(x, y, z1 - 3.6) * Rot(0, 0, a) * Pos(r, 0, 0) * Box(0.8, 2.0, 1.6, align=C)
    out.append(DPart(f"Motor {i} (8520 coreless)", "propulsion", can, STEEL, 0.85, 0.35, f"motor_{i}"))
    cap = Pos(x, y, z0) * Cylinder(r - 0.3, 1.5, align=C)
    out.append(DPart(f"Motor {i} end cap", "propulsion", cap, BLACK, 0.0, 0.7, f"motor_{i}"))
    boss = Pos(x, y, z1 - 0.8) * Cylinder(1.6, 0.8, align=C) + Pos(x, y, z1 - 0.8) * Cylinder(r - 0.5, 0.3, align=C)
    out.append(DPart(f"Motor {i} bearing boss", "propulsion", boss, SILVER, 0.9, 0.3, f"motor_{i}"))
    out.append(DPart(f"Motor {i} shaft (0.8 mm)", "propulsion", Pos(x, y, z1) * Cylinder(0.4, d["prop_z1"] - z1 - 0.6, align=C),
                     SILVER, 0.9, 0.25))
    return out


def zip_tie_motor(spec, i, x, y):
    """Zip tie seated in the tube's V-groove (it fills the groove's root and stands 0.4 mm proud),
    with its locking head beside the clamp slit."""
    g, d = spec["geometry_mm"], dims(spec)
    ro = d["tube_ro"]
    zg = g["motor_z0"] + g["motor_grip_length"] / 2
    k = math.tan(math.radians(50.0)) if False else 1.9 / 1.6   # groove flank slope (dz per dr), as lite.frame
    r_out = ro + 0.4
    hz = (r_out - (ro - 0.6)) * k
    band = revolve(Plane.XZ * Polygon((ro - 0.6, 0), (r_out, -hz), (r_out, hz), align=None), axis=Axis.Z)
    a_in = math.degrees(math.atan2(-y, -x))
    head = Rot(0, 0, a_in + 180 + 35) * Pos(r_out + 1.1, 0, 0) * rbox(-1.1, 1.1, -1.6, 1.6, -1.4, 2.8, rv=0.4)
    head -= Rot(0, 0, a_in + 180 + 35) * Pos(r_out + 1.1, 0, 0) * Box(0.6, 1.3, 4.0, align=(Align.CENTER,) * 3)
    tie = Pos(x, y, zg) * (band + head)
    return [DPart(f"Motor {i} clamp zip tie", "propulsion", tie, NYLON, 0.0, 0.7)]


def _blade(spec, cw: bool):
    """One blade along +x, lofted through elliptical sections with decreasing pitch."""
    d = dims(spec)
    R = d["prop_r"]
    zc = (d["prop_z0"] + d["prop_z1"]) / 2
    stations = [(2.4, 4.0, 1.2, 40), (6.0, 6.8, 0.9, 34), (12.0, 7.2, 0.75, 24),
                (20.0, 6.4, 0.6, 17), (27.0, 5.0, 0.5, 13), (R - 0.4, 2.6, 0.4, 11)]
    secs = []
    for r, chord, t, p in stations:
        th = math.radians(p)
        lead = (0.0, -math.cos(th), math.sin(th)) if cw else (0.0, math.cos(th), math.sin(th))
        pl = Plane(origin=(r, 0, zc), x_dir=lead, z_dir=(1, 0, 0))
        secs.append(pl * Ellipse(chord / 2, t / 2))
    return loft(secs)


def prop_detail(spec, i, x, y):
    d = dims(spec)
    cw = i in CW
    hub = Pos(0, 0, d["prop_z0"]) * Cylinder(2.6, d["prop_z1"] - d["prop_z0"], align=C)
    try:
        hub = fillet(hub.edges().group_by(Axis.Z)[-1], 0.6)
    except Exception:
        pass
    hub -= Pos(0, 0, d["prop_z1"] - 0.5) * Cylinder(1.2, 0.6, align=C)            # moulded dimple on top
    hub -= Pos(0, 0, d["prop_z0"] - 0.1) * Cylinder(0.4, d["prop_z1"] - d["prop_z0"] - 0.5, align=C)  # shaft bore (press fit)
    b = _blade(spec, cw)
    prop = hub + b + Rot(0, 0, 180) * b
    ang = 30 + 47 * i  # parked at different angles so it doesn't look like a render
    prop = Pos(x, y, 0) * Rot(0, 0, ang) * prop
    return [DPart(f"Prop {i} ({'CW' if cw else 'CCW'}, 65 mm 2-blade)", "propulsion", prop,
                  PROP_CW if cw else PROP_CCW, 0.0, 0.5, f"prop_{i}")]


def motor_leads(spec, i, x, y, pads, z1b):
    """Motor leads: down through the ledge's lead hole, out of a side vent window, along the
    arm top, up over the board edge, across the board top and down into the pad holes."""
    g, d = spec["geometry_mm"], dims(spec)
    ro = d["tube_ro"]
    z0m = g["motor_z0"]
    z_lo, z_hi = g["motor_tube_z"]
    h_arm = g["arm_height"]
    r_w = 0.35
    a_in = math.atan2(-y, -x)
    # pick the side window that faces the pads
    pmx = sum(p["x"] for p in pads) / 2
    pmy = sum(p["y"] for p in pads) / 2
    cand = [a_in + math.pi / 2, a_in - math.pi / 2]
    a_w = min(cand, key=lambda a: math.hypot(x + (ro + 2) * math.cos(a) - pmx, y + (ro + 2) * math.sin(a) - pmy))
    # window centre height (as lite.frame._motor_mount)
    rp = ro - g["motor_pedestal_wall"]
    r_lead = d["tube_ri"] - 1.0
    cone_h = (rp - r_lead) * math.tan(math.radians(60))
    cone_z0 = z0m - 1.0 - cone_h
    win_h = min(7.0, cone_z0 - z_lo - 1.5)
    zc = z_lo + 1.0 + win_h / 2
    ux, uy = math.cos(a_in), math.sin(a_in)              # towards the frame centre
    wx, wy = math.cos(a_w), math.sin(a_w)
    tx, ty = -uy, ux
    z_arm = h_arm + r_w + 0.005                          # resting on the arm top
    z_brd = z1b + SOL + r_w + 0.005                      # resting on the board top
    sy = 1.0 if y > 0 else -1.0
    sx = 1.0 if x > 0 else -1.0
    y_edge = sy * 19.75                                  # outside the 38 mm board edge
    pads = sorted(pads, key=lambda p: abs(p["y"]))        # inner pad first
    # of the two ways round in the bore, keep the one where the leads do not touch
    best = None
    for sgn in (1.0, -1.0):
        leads = _motor_lead_pair(i, x, y, pads, sgn, tx, ty, ux, uy, wx, wy, ro, zc, z0m, z_arm, z_brd, z1b, y_edge, sx, r_w)
        try:
            v = (leads[0].part & leads[2].part).volume
        except Exception:
            v = 1e9
        if best is None or v < best[0]:
            best = (v, leads)
    return best[1]


def _motor_lead_pair(i, x, y, pads, sgn, tx, ty, ux, uy, wx, wy, ro, zc, z0m, z_arm, z_brd, z1b, y_edge, sx, r_w):
    out = []
    for k, (p, col) in enumerate(zip(pads, (RED, WIRE_BLK))):
        bo = sgn * (k - 0.5) * 1.2                       # 1.2 mm apart in the bore and window
        zw = 1.2 * (1 - k)                               # stacked in the window (lower lead clears the arm top) ...
        dz = 1.0 * (1 - k)                               # ... and along the arm
        lane = sx * (0.0 if k == 1 else 1.5)              # the run to the inner pad goes 1.5 mm further out
        xr = p["x"] + lane
        pts = [
            # down the bore side by side (1.2 mm apart along the arm), then one above the other out
            # of the side window and along the arm, so the pair never lies in line or crosses
            (x + ux * bo, y + uy * bo, z0m),                                     # leaves the end cap
            (x + ux * bo, y + uy * bo, zc + zw),                                 # down the bore
            (x + wx * (ro + 1.8), y + wy * (ro + 1.8), zc + zw),                 # out of the window
            (x + ux * 13.0, y + uy * 13.0, z_arm + dz),                          # onto the arm
            (xr, y_edge, z_arm + dz),                                            # along the arm / rim to the board edge
            (xr, y_edge, z_brd),                                                 # up past the board edge
            (xr, p["y"], z_brd),                                                 # across the board top
            (p["x"], p["y"], z_brd),
            (p["x"], p["y"], z1b - 1.6 - 0.3),                                   # down through the pad hole
        ]
        # drop consecutive duplicates (lane 0 makes the last two horizontal points coincide)
        clean = [pts[0]]
        for q in pts[1:]:
            if math.dist(q, clean[-1]) > 1e-6:
                clean.append(q)
        w = wire(clean, r=r_w, bend=0.6)
        out.append(DPart(f"Motor {i} lead ({'red' if col == RED else 'black'})", "propulsion", w, col, 0.0, 0.7))
        out.append(DPart(f"Motor {i} lead solder joint", "avionics",
                         solder_fillet(p["x"], p["y"], z1b + SOL, 0.8, r_w, 0.45, up=True) - w, SOLDER, 0.85, 0.3))
    return out


# ------------------------------------------------------------------ board
_SMD = {"SOT-23_Mill": ("sot", 2.92, 1.3, 0.95), "SOT-23-5_Mill": ("sot", 2.9, 1.6, 1.1),
        "R_1206_Mill": ("rc", 3.2, 1.6, 0.55), "C_1206_Mill": ("rc", 3.2, 1.6, 1.0),
        "D_SMA_SOD-123_Mill": ("sod", 2.7, 1.6, 1.1)}


def board_detail(spec):
    g, d = spec["geometry_mm"], dims(spec)
    bx, by, bt = g["board_size"]
    z0, z1 = d["board_z0"], d["board_z1"]
    lay = _layout()
    fps = lay["footprints"]
    hx, hy = g["board_holes"]
    holes = [(sx * hx, sy * hy) for sx in (-1, 1) for sy in (-1, 1)]
    out = []

    # ---- bare board: FR4 core + copper both sides
    fr4 = Pos(0, 0, z0 + CU) * Box(bx, by, bt - 2 * CU, align=C)
    try:
        fr4 = fillet(fr4.edges().filter_by(Axis.Z), 1.0)
    except Exception:
        pass
    drills = []
    for fp in fps:
        for p in _pads(fp):
            if p["drill"]:
                drills.append(Pos(p["x"], p["y"], z0 - 1) * Cylinder(p["drill"] / 2, bt + 2, align=C))
    for hxy in holes:
        fr4 -= Pos(*hxy, z0 - 1) * Cylinder(1.1, bt + 2, align=C)
    fr4 -= _union(drills)
    out.append(DPart("School-milled board (FR4, 46 x 38 mm)", "avionics", fr4, FR4, 0.0, 0.8, "board"))
    for side, zc in (("F", z1 - CU), ("B", z0)):
        cu = Pos(0, 0, zc) * Box(bx - 1.2, by - 1.2, CU, align=C)
        try:
            cu = fillet(cu.edges().filter_by(Axis.Z), 0.6)
        except Exception:
            pass
        iso, islands = [], []
        for fp in fps:
            for p in _pads(fp):
                if p["drill"] or fp["side"] == side:
                    iso.append(_pad_solid(p, zc - 0.01, CU + 0.02, grow=0.4))
                    islands.append(_pad_solid(p, zc, CU))
        for hxy in holes:
            iso.append(Pos(*hxy, zc - 0.01) * Cylinder(2.4, CU + 0.02, align=C))
        cu -= _union(iso)
        cu += _union(islands)
        cu -= _union(drills)
        out.append(DPart(f"Copper pour ({'top' if side == 'F' else 'bottom'}, milled)", "avionics", cu, COPPER, 0.6, 0.45, "board"))

    # ---- tinned pads, per footprint
    for fp in fps:
        pads = _pads(fp)
        if not pads or fp["footprint"].startswith("MountingHole"):
            continue
        tins = []
        for p in pads:
            if p["drill"]:
                for zt in (z1, z0 - SOL):
                    tins.append(_pad_solid(p, zt, SOL) - Pos(p["x"], p["y"], zt - 1) * Cylinder(p["drill"] / 2, SOL + 2, align=C))
            elif fp["side"] == "B":
                tins.append(_pad_solid(p, z0 - SOL, SOL))
            else:
                tins.append(_pad_solid(p, z1, SOL))
        out.append(DPart(f"{fp['ref']} pads (tinned)", "avionics", _union(tins), TIN, 0.8, 0.35))

    # ---- bottom-side SMD parts (hang under the board, standing on their tinned pads)
    zf = z0 - SOL   # pad tin face
    for fp in fps:
        kind = fp["footprint"]
        if kind not in _SMD:
            continue
        fam, L, W, H = _SMD[kind]
        pads = _pads(fp)
        label = f"{fp['ref']}: {fp['value']}"
        if fam == "rc":
            (p1, p2) = pads
            cx, cy = (p1["x"] + p2["x"]) / 2, (p1["y"] + p2["y"]) / 2
            ang = math.degrees(math.atan2(p2["y"] - p1["y"], p2["x"] - p1["x"]))
            te = 0.5
            body_col = CAP_TAN if kind.startswith("C_") else BLACK
            mid = obox(cx, cy, zf - H, L - 2 * te, W - 0.02, H, ang, rt=0.05, rb=0.05)
            ends = None
            for s in (-1, 1):
                ex = cx + s * (L / 2 - te / 2) * math.cos(math.radians(ang))
                ey = cy + s * (L / 2 - te / 2) * math.sin(math.radians(ang))
                e = obox(ex, ey, zf - H, te, W, H, ang, rv=0.08, rb=0.08)
                ends = e if ends is None else ends + e
            out.append(DPart(label, "avionics", mid, body_col, 0.0, 0.6, "board"))
            out.append(DPart(label + " terminations", "avionics", ends, SOLDER, 0.85, 0.3, "board"))
            continue
        if fam == "sot":
            # long axis along the row of pads 1-2(-3); body centred between the two pad rows
            ax = math.atan2(pads[1]["y"] - pads[0]["y"], pads[1]["x"] - pads[0]["x"])
            ca, sa = math.cos(ax), math.sin(ax)
            across = [-p["x"] * sa + p["y"] * ca for p in pads]
            amid = (max(across) + min(across)) / 2
            along = [p["x"] * ca + p["y"] * sa for p in pads]
            lmid = (max(along) + min(along)) / 2
            cx, cy = lmid * ca - amid * sa, lmid * sa + amid * ca
            ang = math.degrees(ax)
            body = obox(cx, cy, zf - 0.05 - H, L, W, H, ang, rv=0.1, rb=0.12)
            feet = [(p["x"], p["y"], 0.55) for p in pads]
            leads = gullwing_leads(cx, cy, ang, W / 2, feet, zf, H, 0.05, down=True)
            out.append(DPart(label, "avionics", body, BLACK, 0.0, 0.55, "board"))
            out.append(DPart(label + " leads", "avionics", leads, SOLDER, 0.85, 0.3, "board"))
            continue
        if fam == "sod":
            (p1, p2) = pads
            cx, cy = (p1["x"] + p2["x"]) / 2, (p1["y"] + p2["y"]) / 2
            ax = math.degrees(math.atan2(p2["y"] - p1["y"], p2["x"] - p1["x"]))
            body = obox(cx, cy, zf - 0.05 - H, L, W, H, ax, rv=0.1, rb=0.12)
            band = obox(cx - (L / 2 - 0.45) * math.cos(math.radians(ax)), cy - (L / 2 - 0.45) * math.sin(math.radians(ax)),
                        zf - 0.05 - H - 0.01, 0.4, W - 0.2, 0.01, ax, rv=0.1)   # cathode band
            feet = [(p["x"], p["y"], 0.7) for p in pads]
            leads = gullwing_leads(cx, cy, ax + 90.0, L / 2, feet, zf, H, 0.05, down=True, w=0.6)
            out.append(DPart(label, "avionics", body, BLACK, 0.0, 0.55, "board"))
            out.append(DPart(label + " cathode band", "avionics", band, SILVER, 0.2, 0.5, "board"))
            out.append(DPart(label + " leads", "avionics", leads, SOLDER, 0.85, 0.3, "board"))

    # ---- through-hole parts on top
    def tht_pins(pads, z_top, s=0.64, tail=1.2, round_=False):
        """Pins from below the board up to z_top, with bottom solder fillets."""
        pins, fil = [], []
        for p in pads:
            if round_:
                pins.append(Pos(p["x"], p["y"], z0 - SOL - tail) * Cylinder(s / 2, z_top - (z0 - SOL - tail), align=C))
                fil.append(solder_fillet(p["x"], p["y"], z0 - SOL, max(p["w"], p["h"]) / 2 - 0.05, s / 2, 0.7, up=False))
            else:
                pins.append(square_pin(p["x"], p["y"], z0 - SOL - tail, z_top, s))
                fil.append(solder_fillet(p["x"], p["y"], z0 - SOL, max(p["w"], p["h"]) / 2 - 0.05, s / 2, 0.7, up=False, square=s))
        return _union(pins), _union(fil)

    zt = z1 + SOL   # top tin face
    for fp in fps:
        kind, ref = fp["footprint"], fp["ref"]
        pads = _pads(fp)
        label = f"{ref}: {fp['value']}"
        if kind.startswith("CP_Radial"):  # 470 uF can on its legs, under the ESP32 edge
            cx, cy = sum(p["x"] for p in pads) / 2, sum(p["y"] for p in pads) / 2
            zb = zt + 0.5
            can = Pos(cx, cy, zb) * Cylinder(3.15, 6.9, align=C)
            try:
                can = fillet(can.edges().group_by(Axis.Z)[-1], 0.35)
            except Exception:
                pass
            groove = revolve(Plane.XZ * Polygon((3.15 - 0.25, 0), (3.2, -0.3), (3.2, 0.3), align=None), axis=Axis.Z)
            can -= Pos(cx, cy, zb + 5.6) * groove
            for a in (0, 90):  # vent cross in the top
                can -= Pos(cx, cy, zb + 6.9 - 0.15) * Rot(0, 0, a) * Box(4.0, 0.3, 0.3, align=C)
            out.append(DPart(label, "avionics", can, 0x1F3A93, 0.15, 0.45, "top"))
            legs, fil = tht_pins(pads, zb, s=0.5, round_=True)
            out.append(DPart(label + " legs", "avionics", legs, SILVER, 0.9, 0.3, "top"))
            out.append(DPart(label + " solder", "avionics", fil, SOLDER, 0.85, 0.3))
        elif kind.startswith("SW_SPDT"):
            cx, cy = sum(p["x"] for p in pads) / 3, sum(p["y"] for p in pads) / 3
            # SS-12D00 body 8.5 x 3.5 mm (datasheet), clear of J1 and the screw head
            base = rbox(cx - 4.25, cx + 4.25, cy - 1.75, cy + 1.75, zt, 0.6, rv=0.3)
            shell = rbox(cx - 4.25, cx + 4.25, cy - 1.75, cy + 1.75, zt + 0.6, 3.0, rv=0.2, rt=0.2)
            shell -= rbox(cx - 2.0, cx + 2.0, cy - 0.8, cy + 0.8, zt + 3.3, 0.5, rv=0.3)
            lever = rbox(cx - 1.5, cx + 0.5, cy - 0.75, cy + 0.75, zt + 3.3, 2.5, rv=0.2, rt=0.2)
            out.append(DPart(label + " base", "avionics", base, BLACK, 0.0, 0.6, "top"))
            out.append(DPart(label, "avionics", shell, SILVER, 0.85, 0.35, "top"))
            out.append(DPart(label + " lever", "avionics", lever, BLACK, 0.0, 0.6, "top"))
            pins, fil = tht_pins(pads, zt, s=0.5)
            out.append(DPart(label + " pins", "avionics", pins, SILVER, 0.9, 0.3, "top"))
            out.append(DPart(label + " solder", "avionics", fil, SOLDER, 0.85, 0.3))
        elif kind.startswith("JST_PH"):
            cx, cy = sum(p["x"] for p in pads) / 2, sum(p["y"] for p in pads) / 2
            Hs = 6.0
            shroud = rbox(cx - 3.0, cx + 3.0, cy - 2.25, cy + 2.25, zt, Hs, rv=0.2, rt=0.15)
            cav = rbox(cx - 2.2, cx + 2.2, cy - 1.45, cy + 1.45, zt + 1.4, Hs, rv=0.15)
            shroud -= cav
            shroud -= rbox(cx - 0.8, cx + 0.8, cy + 1.4, cy + 2.4, zt + 3.0, Hs, rv=0.1)   # latch window
            out.append(DPart(label, "avionics", shroud, NYLON, 0.0, 0.7, "top"))
            tails, fil = tht_pins(pads, zt, s=0.5)
            contacts = _union(square_pin(p["x"], p["y"], zt + 1.4, zt + 1.4 + 3.0, 0.5) for p in pads)
            out.append(DPart(label + " pins", "avionics", tails, GOLD, 0.9, 0.3, "top"))
            out.append(DPart(label + " contacts", "avionics", contacts, GOLD, 0.9, 0.3, "top"))
            out.append(DPart(label + " solder", "avionics", fil, SOLDER, 0.85, 0.3))
            # battery plug in the shroud (fills the cavity, 1 mm proud), wires out of its top
            plug = rbox(cx - 2.2, cx + 2.2, cy - 1.45, cy + 1.45, zt + 1.4 + 3.0, Hs - 1.4 - 3.0 + 1.0, rv=0.15, rt=0.2)
            out.append(DPart("Battery plug (PH2.0)", "battery", plug, NYLON, 0.0, 0.7))
            out.append(("_plug", (cx, cy, zt + Hs + 1.0, [p["x"] for p in pads])))
        elif kind.startswith("LED_D3"):
            cx, cy = sum(p["x"] for p in pads) / 2, sum(p["y"] for p in pads) / 2
            zb = zt + 1.0
            led = Pos(cx, cy, zb) * Cylinder(1.95, 1.0, align=C) + Pos(cx, cy, zb + 1.0) * Cylinder(1.5, 2.8, align=C)
            led += Pos(cx, cy, zb + 3.8) * Sphere(1.5)
            led -= Pos(cx + 1.95, cy, zb - 0.1) * Box(0.6, 4.0, 1.2, align=C)   # cathode flat
            out.append(DPart(label, "avionics", led, 0x2ECC71, 0.0, 0.2, "top", opacity=0.85))
            legs, fil = tht_pins(pads, zb, s=0.5)
            out.append(DPart(label + " legs", "avionics", legs, SILVER, 0.9, 0.3, "top"))
            out.append(DPart(label + " solder", "avionics", fil, SOLDER, 0.85, 0.3))
    plug_info = [o[1] for o in out if isinstance(o, tuple)]
    out = [o for o in out if not isinstance(o, tuple)]

    # ESP32 female headers: one per inner pad row of U1, socket cavities on every pin
    u1 = next(f for f in fps if f["ref"] == "U1")
    hdr = g["esp32_header_height"]
    cols = {}
    for p in _pads(u1):
        cols.setdefault(round(p["x"], 2), []).append(p)
    for x, ps in cols.items():
        ys = [p["y"] for p in ps]
        hb = rbox(x - 1.27, x + 1.27, min(ys) - 1.27, max(ys) + 1.27, zt, z1 + hdr - zt, rv=0.15, rt=0.1)
        for p in ps:
            hb -= Pos(p["x"], p["y"], z1 + hdr - 6.0) * Box(1.0, 1.0, 6.1, align=C)
        out.append(DPart(f"Female header ({len(ps)}-pin, 8.5 mm)", "avionics", hb, BLACK, 0.0, 0.7, "top"))
        pins, fil = tht_pins(ps, zt)
        out.append(DPart(f"Female header tails ({len(ps)}-pin)", "avionics", pins, GOLD, 0.9, 0.3, "top"))
        out.append(DPart(f"Female header solder ({len(ps)}-pin)", "avionics", fil, SOLDER, 0.85, 0.3))

    # M2 pan-head screws holding the board to the frame bosses
    for (sx_, sy_) in holes:
        head = Pos(sx_, sy_, z1 - CU) * Cylinder(1.75, 1.3, align=C)
        try:
            head = fillet(head.edges().group_by(Axis.Z)[-1], 0.5)
        except Exception:
            pass
        head -= Pos(sx_, sy_, z1 - CU + 0.6) * Rot(0, 0, 45) * Box(2.2, 0.5, 1.0, align=C)
        head -= Pos(sx_, sy_, z1 - CU + 0.6) * Rot(0, 0, -45) * Box(2.2, 0.5, 1.0, align=C)
        shank = Pos(sx_, sy_, d["t"] + 2.0) * Cylinder(g["board_hole"] / 2, z1 - CU - d["t"] - 2.0, align=C)
        out.append(DPart("M2 x 12 pan-head screw", "avionics", head + shank, STEEL, 0.85, 0.35))
    return out, _layout_pads(fps), plug_info


def _layout_pads(fps):
    return {fp["ref"]: _pads(fp) for fp in fps}


# ------------------------------------------------------------------ modules
def esp32_detail(spec, u1_pads):
    g, d = spec["geometry_mm"], dims(spec)
    em = g["esp32_module"]
    cx, cy = em["centre"]
    sx, sy = em["size"]
    z = d["board_z1"] + g["esp32_header_height"]
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    pcb = rbox(x0, x1, y0, y1, z, 1.0, rv=1.2)
    for p in u1_pads:
        pcb -= Pos(p["x"], p["y"], z - 0.1) * Cylinder(0.5, 1.2, align=C)
    out = [DPart("ESP32 MiniKit PCB", "avionics", pcb, BLUE_PCB, 0.1, 0.55, "esp32")]
    zt = z + 1.0
    # WROOM-32: 18 x 25.5 mm, antenna end flush with the +y edge
    wx0, wx1, wy0, wy1 = cx - 9, cx + 9, y1 - 25.5, y1
    out.append(DPart("ESP32-WROOM-32 module", "avionics", rbox(wx0, wx1, wy0, wy1, zt, 0.8, rv=0.3), BLACK_PCB, 0.1, 0.6, "esp32"))
    can = rbox(wx0 + 0.6, wx1 - 0.6, wy0 + 0.6, wy0 + 18.0, zt + 0.8, 2.4, rv=0.5, rt=0.35)
    can -= rbox(wx0 + 2.0, wx1 - 2.0, wy0 + 2.0, wy0 + 16.6, zt + 0.8 + 2.4 - 0.08, 0.1, rv=0.5)   # embossed label panel
    out.append(DPart("ESP32-WROOM-32 shield can", "avionics", can, SILVER, 0.9, 0.3, "esp32"))
    # PCB antenna: a meander trace on the module end
    ant = []
    ya, yb = wy1 - 5.5, wy1 - 1.0
    xs = [wx0 + 1.5 + 1.6 * k for k in range(10)]
    for k, xa in enumerate(xs):
        ant.append(Pos(xa, (ya + yb) / 2, zt + 0.8) * Box(0.4, yb - ya, 0.04, align=C))
        if k < len(xs) - 1:
            yy = yb - 0.2 if k % 2 == 0 else ya + 0.2
            ant.append(Pos(xa + 0.8, yy, zt + 0.8) * Box(2.0, 0.4, 0.04, align=C))
    out.append(DPart("PCB antenna", "avionics", _union(ant), GOLD, 0.7, 0.4, "esp32"))
    # micro-USB receptacle: shell with its plug opening facing -y (the module edge)
    usb = rbox(cx - 3.75, cx + 3.75, y0 + 0.01, y0 + 5.0, zt, 2.5, rv=0.6, rt=0.6)
    usb -= rbox(cx - 3.3, cx + 3.3, y0 - 0.1, y0 + 4.0, zt + 0.45, 1.6, rv=0.5)
    out.append(DPart("Micro-USB (programming)", "avionics", usb, SILVER, 0.9, 0.3, "esp32"))
    out.append(DPart("CP2104 USB-UART (QFN-24)", "avionics", rbox(cx - 2, cx + 2, y0 + 7.0, y0 + 11.0, zt, 0.9, rv=0.15, rt=0.1),
                     BLACK, 0.0, 0.5, "esp32"))
    reg = rbox(cx + 6, cx + 9, y0 + 6.5, y0 + 8.3, zt + 0.1, 1.1, rv=0.15, rt=0.1)
    out.append(DPart("3.3 V regulator (SOT-223)", "avionics", reg, BLACK, 0.0, 0.5, "esp32"))
    out.append(DPart("Regulator tab", "avionics", rbox(cx + 6.6, cx + 8.4, y0 + 8.3, y0 + 9.3, zt, 0.15, rv=0.1), SOLDER, 0.85, 0.3, "esp32"))
    for k in range(2):  # reset / boot tact switches with round actuators
        bx_ = x0 + 2.5 + 4.5 * k
        sw = rbox(bx_, bx_ + 3.0, y0 + 2.0, y0 + 6.0, zt, 1.0, rv=0.2)
        out.append(DPart("Tact switch", "avionics", sw, SILVER, 0.8, 0.35, "esp32"))
        out.append(DPart("Tact switch button", "avionics", Pos(bx_ + 1.5, y0 + 4.0, zt + 1.0) * Cylinder(0.75, 0.5, align=C),
                         BLACK, 0.0, 0.6, "esp32"))
    # male pins: through the module, into the female header cavities; soldered on top
    pins, fil = [], []
    for p in u1_pads:
        pins.append(square_pin(p["x"], p["y"], z - 5.5, zt + 0.9))
        fil.append(solder_fillet(p["x"], p["y"], zt, 0.75, 0.32, 0.5, up=True, square=0.64))
    out.append(DPart("ESP32 MiniKit pins", "avionics", _union(pins), GOLD, 0.9, 0.3))
    out.append(DPart("ESP32 MiniKit pin solder", "avionics", _union(fil), SOLDER, 0.85, 0.3, "esp32"))
    return out


def imu_detail(spec, u2_pads):
    g, d = spec["geometry_mm"], dims(spec)
    im = g["imu_module"]
    z1 = d["board_z1"]
    zt = z1 + SOL
    x0, x1, y0, y1 = im["x"][0] + 0.2, im["x"][1] - 0.2, im["y"][0] + 0.2, im["y"][1] - 0.2
    px = u2_pads[0]["x"]
    ys = [p["y"] for p in u2_pads]
    zp = z1 + 1.5                                   # GY-521 PCB bottom
    spacer = rbox(px - 1.25, px + 1.25, min(ys) - 1.27, max(ys) + 1.27, zt, zp - zt, rv=0.2)
    pcb = rbox(x0, x1, y0, y1, zp, 1.6, rv=0.8)
    for hy_ in (y0 + 1.9, y1 - 1.9):              # the GY-521's two mounting holes
        pcb -= Pos(x1 - 1.9, hy_, zp - 0.1) * Cylinder(1.5, 1.8, align=C)
    for p in u2_pads:
        spacer -= Pos(p["x"], p["y"], zt - 0.1) * Box(0.64, 0.64, zp - zt + 0.2, align=C)
        pcb -= Pos(p["x"], p["y"], zp - 0.1) * Cylinder(0.5, 1.8, align=C)
    out = [DPart("GY-521 pin header", "avionics", spacer, BLACK, 0.0, 0.7, "imu"),
           DPart("GY-521 (MPU-6050) PCB", "avionics", pcb, BLUE_PCB, 0.1, 0.55, "imu")]
    zc = zp + 1.6
    mx, my = (x0 + x1) / 2 + 1.0, (y0 + y1) / 2
    out.append(DPart("MPU-6050 (QFN-24)", "avionics", rbox(mx - 2, mx + 2, my - 2, my + 2, zc, 0.9, rv=0.15, rt=0.1), BLACK, 0.0, 0.45, "imu"))
    out.append(DPart("GY-521 regulator (SOT-23-5)", "avionics", rbox(mx - 1.45, mx + 1.45, y1 - 5, y1 - 3.4, zc, 0.9, rv=0.1, rt=0.1),
                     BLACK, 0.0, 0.5, "imu"))
    for k in range(4):   # 0603 passives
        out.append(DPart("0603 capacitor", "avionics", rbox(mx - 3.5 + 1.8 * k, mx - 2.7 + 1.8 * k, y0 + 1.0, y0 + 2.6, zc, 0.8, rv=0.08),
                         CAP_TAN, 0.0, 0.6, "imu"))
    out.append(DPart("Power LED (0603)", "avionics", rbox(x0 + 4.0, x0 + 4.8, y1 - 2.4, y1 - 0.8, zc, 0.5, rv=0.08), 0xE74C3C, 0.0, 0.3, "imu"))
    pins, fil, top = [], [], []
    for p in u2_pads:
        pins.append(square_pin(p["x"], p["y"], z1 - 1.6 - SOL - 1.2, zc + 0.9))
        fil.append(solder_fillet(p["x"], p["y"], z1 - 1.6 - SOL, 0.85, 0.32, 0.7, up=False, square=0.64))
        top.append(solder_fillet(p["x"], p["y"], zc, 0.75, 0.32, 0.5, up=True, square=0.64))
    out.append(DPart("GY-521 header pins", "avionics", _union(pins), GOLD, 0.9, 0.3))
    out.append(DPart("GY-521 pin solder (board)", "avionics", _union(fil), SOLDER, 0.85, 0.3))
    out.append(DPart("GY-521 pin solder (module)", "avionics", _union(top), SOLDER, 0.85, 0.3, "imu"))
    return out


def charger_detail(spec, u3_pads):
    g, d = spec["geometry_mm"], dims(spec)
    cm = g["charger_module"]
    z1 = d["board_z1"]
    x0, x1, y0, y1 = cm["x"][0] + 0.2, cm["x"][1] - 0.2, cm["y"][0] + 0.2, cm["y"][1] - 0.2
    zp = z1 + 0.6
    pcb = rbox(x0, x1, y0, y1, zp, 1.2, rv=0.6)
    for p in u3_pads:
        pcb -= Pos(p["x"], p["y"], zp - 0.1) * Cylinder(0.5, 1.4, align=C)
    out = [DPart("TP4056 charger PCB", "avionics", pcb, BLUE_PCB, 0.1, 0.55, "charger")]
    zc = zp + 1.2
    yc = (y0 + y1) / 2
    usb = rbox(x1 - 5.5, x1 + 0.0, yc - 3.75, yc + 3.75, zc, 2.6, rv=0.6, rt=0.6)
    usb -= rbox(x1 - 4.5, x1 + 0.1, yc - 3.3, yc + 3.3, zc + 0.45, 1.7, rv=0.5)
    out.append(DPart("Micro-USB (charging, faces the nose)", "avionics", usb, SILVER, 0.9, 0.3, "charger"))
    # TP4056 in SOP-8 with gull-wing leads, DW01 + FS8205 protection, LEDs
    sx_, sy_ = x0 + 11.5, yc
    out.append(DPart("TP4056 (SOP-8)", "avionics", obox(sx_, sy_, zc + 0.1, 4.9, 3.9, 1.45, 0, rv=0.15, rt=0.15), BLACK, 0.0, 0.5, "charger"))
    feet = [(sx_ - 1.905 + 1.27 * k, sy_ + s * 2.7, 0.6) for k in range(4) for s in (-1, 1)]
    out.append(DPart("TP4056 leads", "avionics", gullwing_leads(sx_, sy_, 0.0, 1.95, feet, zc, 1.45, 0.1, down=False),
                     SOLDER, 0.85, 0.3, "charger"))
    for k, (xx, nm, L_, W_) in enumerate(((x0 + 2.4, "DW01A (SOT-23-6)", 2.9, 1.6), (x0 + 6.2, "FS8205A (TSSOP-8)", 3.0, 4.4))):
        out.append(DPart(nm, "avionics", obox(xx, yc, zc, L_, W_, 1.0, 90, rv=0.1, rt=0.1), BLACK, 0.0, 0.5, "charger"))
    for k, col in ((0, RED), (1, 0x2980B9)):
        out.append(DPart("Charge LED (0603)", "avionics", rbox(x0 + 16 + 2.2 * k, x0 + 17.6 + 2.2 * k, y0 + 1.3, y0 + 2.1, zc, 0.5, rv=0.08),
                         col, 0.0, 0.3, "charger"))
    pins, fil, top = [], [], []
    for p in u3_pads:
        pins.append(square_pin(p["x"], p["y"], z1 - 1.6 - SOL - 1.0, zc + 0.6))
        fil.append(solder_fillet(p["x"], p["y"], z1 - 1.6 - SOL, 1.05, 0.32, 0.7, up=False, square=0.64))
        top.append(solder_fillet(p["x"], p["y"], zc, 0.8, 0.32, 0.5, up=True, square=0.64))
    out.append(DPart("TP4056 module pins", "avionics", _union(pins), GOLD, 0.9, 0.3))
    out.append(DPart("TP4056 pin solder (board)", "avionics", _union(fil), SOLDER, 0.85, 0.3))
    out.append(DPart("TP4056 pin solder (module)", "avionics", _union(top), SOLDER, 0.85, 0.3, "charger"))
    return out


def battery_detail(spec, plug_info):
    g, d = spec["geometry_mm"], dims(spec)
    P = spec["parts"]["battery"]
    e = P["envelope_mm"]
    xo, t = P.get("x_offset", 0.0), d["t"]
    z = t + 0.5
    out = []
    # anti-slip foam pad between the plate and the cell
    pad = rbox(xo - e["x"] / 2 + 4, xo + e["x"] / 2 - 4, -e["y"] / 2 + 1.5, e["y"] / 2 - 1.5, t, 0.5, rv=1.0)
    out.append(DPart("Anti-slip foam pad", "battery", pad, FOAM, 0.0, 0.9))
    body = Pos(xo + 1.5, 0, z) * Box(e["x"] - 3.0, e["y"], e["z"] - 0.02, align=C)
    body = fillet(body.edges(), 1.6)
    out.append(DPart("1S 600 mAh LiPo", "battery", body, LIPO, 0.3, 0.5, "battery"))
    xb0 = xo - e["x"] / 2
    band = rbox(xb0, xb0 + 3.0, -(e["y"] - 4) / 2, (e["y"] - 4) / 2, z + 1.5, e["z"] - 3, rv=0.6)
    out.append(DPart("Cell tabs (sealed)", "battery", band, SILVER, 0.8, 0.35, "battery"))
    label = obox(xo + 3, 0, z + e["z"] - 0.02, e["x"] - 14, e["y"] - 6, 0.02, rv=1.0)
    out.append(DPart("Battery label", "battery", label, 0xD4AC0D, 0.0, 0.6, "battery"))
    # strap: over the cell, down its sides, through the plate slots, joined under the plate
    sw, sh = g["strap_slot"]
    ys = g["strap_slot_y"]
    zt = z + e["z"]
    th = 0.8
    wy = e["y"] / 2                       # cell side
    ws = sw - 2.0                         # strap width (10 mm in 12 mm slots)
    yo = ys + sh / 2 - 0.3                # strap leg outer face, inside the slot
    top = rbox(-ws / 2, ws / 2, -yo, yo, zt, th, rv=0.0)
    legs = [rbox(-ws / 2, ws / 2, wy, yo, -th, zt + 2 * th), rbox(-ws / 2, ws / 2, -yo, -wy, -th, zt + 2 * th)]
    bottom = rbox(-ws / 2, ws / 2, -yo, yo, -th, th)
    strap = top + legs[0] + legs[1] + bottom
    strap -= rbox(-ws / 2 - 1, ws / 2 + 1, -yo + th, yo - th, -th + th, zt - (-th + th))  # hollow: the loop
    out.append(DPart("Battery strap (hook-and-loop)", "battery", strap, STRAP, 0.0, 0.9))
    # battery lead: from the tab end, behind the cell, up past the board edge, over to the plug
    for (cx, cy, ztop, pxs) in plug_info:
        for k, (px, col) in enumerate(sorted(zip(pxs, (RED, WIRE_BLK)))):
            yk = -2.5 + 2.0 * k                     # leaves the tab band side by side
            r = 0.4
            # risers 1 mm apart in x; the far pin's lead runs 1 mm to the side, past the end of the
            # near lead's run, then steps over to its pin; both enter the plug vertically
            xr, yr, zh = xb0 - 0.9 - 1.0 * k, cy + 1.0 * k, ztop + 0.8
            pts = [(xb0, yk, z + 4.0), (xr, yk, z + 4.0), (xr, yk, z + 7.0), (xr, yr, z + 9.0),
                   (xr, yr, zh), (px, yr, zh), (px, cy, zh), (px, cy, ztop)]
            out.append(DPart(f"Battery lead ({'red' if col == RED else 'black'})", "battery", wire(pts, r=r, bend=0.4), col, 0.0, 0.7))
    return out


def autonomy_detail(spec):
    """v3: PMW3901 flow and VL53L1X ToF breakouts taped to the tail-tab rims, lens / sensor
    looking down through the windows; XIAO ESP32-S3 Sense stack behind the nose arch, its
    camera lens through the arch's lens hole, tied to the arch."""
    g, d = spec["geometry_mm"], dims(spec)
    au = g.get("autonomy", {})
    if not au.get("fit"):
        return []
    out, t = [], d["t"]
    rim = au["tail_tab"]["rim"]
    for key, owner, label, col in (("flow_module", "flow", "PMW3901 optical-flow breakout", PURPLE_PCB),
                                   ("tof_module", "tof", "VL53L1X time-of-flight breakout", BLUE_PCB)):
        m = au[key]
        (sx, sy, sz), (cx, cy) = m["size"], m["centre"]
        xa, xb, ya, yb = cx - sx / 2 + 0.1, cx + sx / 2 - 0.1, cy - sy / 2 + 0.1, cy + sy / 2 - 0.1
        tape = 0.8
        for s in (-1, 1):   # two strips of foam tape on the long rims
            yy0 = cy + s * (sy / 2 - rim) if s < 0 else cy + sy / 2 - rim
            ty0, ty1 = (cy - sy / 2 + 0.2, cy - sy / 2 + rim - 0.1) if s < 0 else (cy + sy / 2 - rim + 0.1, cy + sy / 2 - 0.2)
            out.append(DPart("Foam tape", "autonomy", rbox(xa + 0.5, xb - 0.5, ty0, ty1, t, tape, rv=0.3), FOAM, 0.0, 0.9, owner))
        zp = t + tape
        pcb = rbox(xa, xb, ya, yb, zp, 1.0, rv=1.0)
        for hx_ in (xa + 1.6, xb - 1.6):
            pcb -= Pos(hx_, cy + (sy / 2 - 2.0) * (1 if owner == "flow" else -1), zp - 0.1) * Cylinder(0.8, 1.2, align=C)
        out.append(DPart(label, "autonomy", pcb, col, 0.1, 0.55, owner))
        # header pins along one end (where the lead plugs in)
        hdr = rbox(xb - 2.6, xb - 0.4, cy - 3.8, cy + 3.8, zp + 1.0, sz - tape - 1.0 - 0.1, rv=0.2)
        out.append(DPart(label + " header", "autonomy", hdr, BLACK, 0.0, 0.7, owner))
        if owner == "flow":   # sensor + lens holder hang under the PCB, into the window
            holder = Pos(cx, cy, zp - 2.0) * Cylinder(2.6, 2.0, align=C)
            holder -= Pos(cx, cy, zp - 2.1) * Cylinder(1.5, 0.4, align=C)
            out.append(DPart("PMW3901 + lens holder", "autonomy", holder, BLACK, 0.0, 0.3, owner))
            out.append(DPart("Flow lens", "autonomy", Pos(cx, cy, zp - 2.0) * Cylinder(1.4, 0.3, align=C), 0x5DADE2, 0.3, 0.1, owner, 0.8))
        else:
            chip = rbox(cx - 2.45, cx + 2.45, cy - 1.25, cy + 1.25, zp - 1.56, 1.56, rv=0.1, rb=0.1)
            out.append(DPart("VL53L1X (optical LGA)", "autonomy", chip, BLACK, 0.0, 0.4, owner))
            for s in (-1, 1):
                out.append(DPart("VL53L1X window", "autonomy", Pos(cx + s * 1.1, cy, zp - 1.56 - 0.02) * Cylinder(0.45, 0.02, align=C),
                                 0x5DADE2, 0.3, 0.1, owner, 0.8))
    # ---- XIAO ESP32-S3 Sense stack (boards stand vertically, camera looking forward)
    cam = au["camera"]
    a = au["arch"]
    cy_, cz_, cdep = cam["size"]
    xA = a["x0"]                  # arch back face
    x0 = xA - cdep
    yc, z0 = cam["y_centre"], cam["z0"]
    ylo, yhi, zlo, zhi = yc - cy_ / 2 + 0.1, yc + cy_ / 2 - 0.1, z0 + 0.1, z0 + cz_ - 0.1
    zl = a["lens_z"]
    vplate = lambda xa, xb, col, nm, r=1.0: DPart(nm, "autonomy", Pos(xa, 0, 0) * (Rot(0, 90, 0) * rbox(-zhi, -zlo, ylo, yhi, 0, xb - xa, rv=r)),
                                                  col, 0.1, 0.55, "camera")
    main = vplate(x0 + 0.2, x0 + 1.4, BLUE_PCB, "XIAO ESP32-S3 (main board)")
    out.append(main)
    sh = Pos(x0 + 1.4, 0, 0) * (Rot(0, 90, 0) * rbox(-(zlo + 14.0), -(zlo + 1.0), yc - 6.25, yc + 6.25, 0, 2.0, rv=0.4, rt=0.3))
    out.append(DPart("ESP32-S3 module shield", "autonomy", sh, SILVER, 0.9, 0.3, "camera"))
    usb = Pos(x0 + 1.4, 0, 0) * (Rot(0, 90, 0) * rbox(-(zhi - 0.1), -(zhi - 3.3), yc - 4.45, yc + 4.45, 0, 3.2, rv=1.2))
    out.append(DPart("USB-C (programming)", "autonomy", usb, SILVER, 0.9, 0.3, "camera"))
    sense = vplate(x0 + 6.0, x0 + 7.2, BLACK_PCB, "Sense expansion board")
    out.append(sense)
    b2b = Pos(x0 + 1.4, 0, 0) * (Rot(0, 90, 0) * rbox(-(zlo + 16.5), -(zlo + 14.5), yc - 3.0, yc + 3.0, 0, x0 + 6.0 - (x0 + 1.4), rv=0.3))
    out.append(DPart("Board-to-board connector", "autonomy", b2b, BLACK, 0.0, 0.6, "camera"))
    # OV2640 module on the Sense board, lens barrel through the arch's lens hole
    mod = Pos(x0 + 7.2, 0, 0) * (Rot(0, 90, 0) * rbox(-(zl + 4.25), -(zl - 4.25), yc - 4.25, yc + 4.25, 0, 4.0, rv=0.6))
    out.append(DPart("OV2640 camera module", "autonomy", mod, BLACK, 0.0, 0.5, "camera"))
    r_lens = a["lens_hole_r"] - 0.3
    lens = xcyl(x0 + 11.2, yc, zl, r_lens, xA + a["thickness"] + 0.6 - (x0 + 11.2))
    lens -= xcyl(xA + a["thickness"] + 0.3, yc, zl, 1.8, 0.4)
    out.append(DPart("Camera lens barrel", "autonomy", lens, DARK, 0.2, 0.25, "camera"))
    out.append(DPart("Camera lens glass", "autonomy", xcyl(xA + a["thickness"] + 0.3, yc, zl, 1.8, 0.3), 0x5DADE2, 0.3, 0.1, "camera", 0.8))
    # zip tie round the stack: behind the main board, through the arch's two slots, across its front
    zc = z0 + cz_ / 2
    sw_, sh_ = a["tie_slot"]
    yA, yB = yc - cy_ / 2 - 1.5, yc + cy_ / 2 + 1.5
    tt, tw = 0.9, 3.0
    xr = x0 + 0.2                          # main board back face
    xf = xA + a["thickness"]               # arch front face
    loop = rbox(xr - tt, xf + tt, yA - tt / 2, yB + tt / 2, zc - tw / 2, tw)
    loop -= rbox(xr, xf, yA + tt / 2, yB - tt / 2, zc - tw / 2 - 1, tw + 2)
    head = rbox(xf + tt, xf + tt + 2.4, yc - 1.5, yc + 1.5, zc - tw / 2 - 0.6, tw + 0.3, rv=0.3)
    out.append(DPart("Camera zip tie", "autonomy", loop + head, NYLON, 0.0, 0.7))
    return out


# ------------------------------------------------------------------ assembly
def build_detail(spec=None, with_guard=True):
    spec = spec or load_spec()
    g, d = spec["geometry_mm"], dims(spec)
    parts = [DPart("Frame (one-piece PETG print)", "frame", frame(spec), PETG, 0.0, 0.55)]
    board, pads, plug_info = board_detail(spec)
    motor_pads = {}
    for ref in ("M1", "M2", "M3", "M4"):
        ps = pads[ref]
        mx, my = sum(p["x"] for p in ps) / 2, sum(p["y"] for p in ps) / 2
        motor_pads[ref] = (mx, my, ps)
    for i, (x, y) in enumerate(motor_xy(spec)):
        parts += motor_detail(spec, i, x, y)
        parts += prop_detail(spec, i, x, y)
        parts += zip_tie_motor(spec, i, x, y)
        ref = min(motor_pads, key=lambda r: math.hypot(motor_pads[r][0] - x, motor_pads[r][1] - y))
        parts += motor_leads(spec, i, x, y, motor_pads[ref][2], d["board_z1"])
    parts += board
    parts += esp32_detail(spec, pads["U1"]) + imu_detail(spec, pads["U2"]) + charger_detail(spec, pads["U3"])
    parts += battery_detail(spec, plug_info)
    parts += autonomy_detail(spec)
    if with_guard:
        parts.append(DPart(f"Clip-on prop guard (optional, {g['guard']['height']:.0f} mm PETG)", "guard", guard(spec), PETG_GUARD, 0.0, 0.6))
    for i, (x, y) in enumerate(motor_xy(spec)):
        parts.append(DPart(f"Prop {i} swept disc", "sweep", prop_disk(spec, x, y), 0x2E86C1, 0.0, 0.6, opacity=0.15))
    return parts


def fit_check(spec, parts, tol=0.02):
    """Detailed parts vs the envelopes the checks used. Returns a list of problems."""
    env = {c.name: c.part for c in build_assembly(spec)}
    env.update({f"prop_{i}": prop_disk(spec, x, y) for i, (x, y) in enumerate(motor_xy(spec))})
    probs = []
    for p in parts:
        if p.owner in env:
            outside = (p.part - env[p.owner]).volume
            if outside > tol * max(1.0, p.part.volume * 0.001) and outside > tol:
                probs.append(f"{p.name}: {outside:.3f} mm^3 outside the {p.owner} envelope")
    tops = [p for p in parts if p.owner == "top"]
    for t in tops:
        for name, e in env.items():
            if name in ("board",):
                continue
            ov = (t.part & e).volume
            if ov > tol:
                probs.append(f"{t.name} intersects {name}: {ov:.3f} mm^3")
    return probs


def main():
    spec = load_spec()
    parts = build_detail(spec)
    print(f"{len(parts)} detailed parts")
    probs = fit_check(spec, parts)
    print("FIT", "PASS" if not probs else "FAIL", *probs, sep="\n  ")
    raise SystemExit(1 if probs else 0)


if __name__ == "__main__":
    main()

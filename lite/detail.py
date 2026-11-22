"""NF-3D Lite detailed visual model (for the browser viewer and renders only).

The checks in lite/checks.py run on simple envelopes (a cylinder per motor, a box per
module), which is what they need. This file builds what the parts actually look like,
from the same spec numbers and the board's real layout (lite/ecad/layout/build/board.json):

  motors     can with vent slots, black end cap, bearing boss, shaft, two leads
  props      hub + two lofted, twisted blades; CW and CCW props are mirror images
             (motors 0 and 2 spin clockwise seen from above, 1 and 3 anticlockwise)
  board      FR4 with copper pour, mounting holes, every footprint from the layout
             (top side and bottom side), female headers for the ESP32
  modules    ESP32 MiniKit (PCB, WROOM module with shield can and antenna, USB, USB-UART),
             GY-521 (PCB, MPU-6050, header), TP4056 (PCB, USB towards the nose, chip, LEDs)
  battery    1S pouch with rounded edges, tab end band and leads

    python -m lite.detail          prints a part count and runs the fit check:
every detailed part must stay inside the envelope the checks used for it (so the
clearance numbers in checks.json still hold for the detailed model), and the top-side
board parts that have no envelope must not hit anything.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass

from build123d import (Align, Box, Cylinder, Ellipse, Part, Plane, Pos, Rot, Sphere, fillet, loft, mirror)

from .frame import C, MOTOR_SIGNS, build_assembly, dims, frame, guard, load_spec, motor_xy, prop_disk

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD_JSON = os.path.join(HERE, "ecad", "layout", "build", "board.json")
CW = {0, 2}  # BUILD.md motor direction table

# colours (sRGB hex) and finish
PETG, PETG_GUARD = 0xE67E22, 0x5D6D7E
STEEL, SILVER, BLACK, DARK = 0xB8BCC2, 0xD0D3D6, 0x1C1C1E, 0x2B2F33
COPPER, FR4, SOLDER = 0xC98A4B, 0xD9C68E, 0xC9CCCF
BLUE_PCB, BLACK_PCB = 0x1F5AA6, 0x202326
PROP_CW, PROP_CCW = 0xD5D8DC, 0x34495E
RED, WIRE_BLK, CAP_TAN, LIPO = 0xC0392B, 0x222222, 0x9C7A4F, 0xC9D1D8


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


def _box(x0, x1, y0, y1, z0, h):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0, y1 - y0, h, align=C)


# ------------------------------------------------------------------ motors and props
def motor_detail(spec, i, x, y):
    g, d = spec["geometry_mm"], dims(spec)
    r = spec["parts"]["motor"]["envelope_mm"]["diameter"] / 2 - 0.05  # as lite.frame.motor()
    z0, z1 = g["motor_z0"], d["motor_z1"]
    out = []
    can = Pos(x, y, z0 + 1.5) * Cylinder(r, z1 - z0 - 2.3, align=C)
    for k in range(4):  # vent slots near the top of the can
        a = 45 + 90 * k
        can -= Pos(x, y, z1 - 3.6) * Rot(0, 0, a) * Pos(r, 0, 0) * Box(0.8, 2.0, 1.6, align=C)
    out.append(DPart(f"Motor {i} (8520 coreless)", "propulsion", can, STEEL, 0.85, 0.35, f"motor_{i}"))
    out.append(DPart(f"Motor {i} end cap", "propulsion", Pos(x, y, z0) * Cylinder(r - 0.3, 1.5, align=C), BLACK, 0.0, 0.7, f"motor_{i}"))
    boss = Pos(x, y, z1 - 0.8) * Cylinder(1.6, 0.8, align=C) + Pos(x, y, z1 - 0.8) * Cylinder(r - 0.5, 0.3, align=C)
    out.append(DPart(f"Motor {i} bearing boss", "propulsion", boss, SILVER, 0.9, 0.3, f"motor_{i}"))
    out.append(DPart(f"Motor {i} shaft (0.8 mm)", "propulsion", Pos(x, y, z1) * Cylinder(0.4, d["prop_z1"] - z1 - 0.3, align=C), SILVER, 0.9, 0.25))
    for k, col in ((-1, RED), (1, WIRE_BLK)):  # leads leave the end cap downwards inside the tube bore
        out.append(DPart(f"Motor {i} lead", "propulsion", Pos(x + k * 1.6, y, z0 - 3.0) * Cylinder(0.35, 3.0, align=C), col, 0.0, 0.8))
    return out


def _blade(spec, cw: bool):
    """One blade along +x, lofted through elliptical sections with decreasing pitch."""
    d = dims(spec)
    R = d["prop_r"]
    zc = (d["prop_z0"] + d["prop_z1"]) / 2
    # (radius, chord, thickness, pitch deg): broad root, narrow tip, ~40 mm geometric pitch
    stations = [(2.0, 4.0, 1.2, 40), (6.0, 6.8, 0.9, 34), (12.0, 7.2, 0.75, 24),
                (20.0, 6.4, 0.6, 17), (27.0, 5.0, 0.5, 13), (R - 0.4, 2.6, 0.4, 11)]
    secs = []
    for r, chord, t, p in stations:
        th = math.radians(p)
        # trailing -> leading edge direction. A clockwise (from above) prop moves its +x
        # blade towards -y, and the leading edge sits high so the air is pushed down.
        lead = (0.0, -math.cos(th), math.sin(th)) if cw else (0.0, math.cos(th), math.sin(th))
        pl = Plane(origin=(r, 0, zc), x_dir=lead, z_dir=(1, 0, 0))
        secs.append(pl * Ellipse(chord / 2, t / 2))
    return loft(secs)


def prop_detail(spec, i, x, y):
    d = dims(spec)
    cw = i in CW
    hub = Pos(0, 0, d["prop_z0"]) * Cylinder(2.6, d["prop_z1"] - d["prop_z0"], align=C)
    hub -= Pos(0, 0, d["prop_z1"] - 0.6) * Cylinder(1.2, 0.6, align=C)  # moulded dimple on top
    b = _blade(spec, cw)
    prop = hub + b + Rot(0, 0, 180) * b
    ang = 30 + 47 * i  # parked at different angles so it doesn't look like a render
    prop = Pos(x, y, 0) * Rot(0, 0, ang) * prop
    return [DPart(f"Prop {i} ({'CW' if cw else 'CCW'}, 65 mm 2-blade)", "propulsion", prop,
                  PROP_CW if cw else PROP_CCW, 0.0, 0.5, f"prop_{i}")]


# ------------------------------------------------------------------ board
# heights of the footprint families in the layout (datasheet body heights, mm)
_SMD_H = {"SOT-23_Mill": 1.1, "SOT-23-5_Mill": 1.1, "R_1206_Mill": 0.6, "C_1206_Mill": 1.0,
          "D_SMA_SOD-123_Mill": 1.25, "SolderJumper_2_Open_Mill": 0.0}


def board_detail(spec):
    g, d = spec["geometry_mm"], dims(spec)
    bx, by, bt = g["board_size"]
    z0, z1 = d["board_z0"], d["board_z1"]
    lay = json.load(open(BOARD_JSON))
    out = []
    fr4 = Pos(0, 0, z0 + 0.04) * Box(bx, by, bt - 0.08, align=C)
    cu = Pos(0, 0, z0) * Box(bx - 1.2, by - 1.2, bt, align=C)  # copper pour both sides, pulled back from the edge
    hx, hy = g["board_holes"]
    for sx in (-1, 1):
        for sy in (-1, 1):
            hole = Pos(sx * hx, sy * hy, z0) * Cylinder(1.1, bt, align=C)
            fr4 -= hole
            cu -= Pos(sx * hx, sy * hy, z0) * Cylinder(2.4, bt, align=C)
    # isolation channels the mill cuts round every pad
    for fp in lay["footprints"]:
        for p in fp["pads"]:
            if p.get("npth"):
                continue
            px, py = _b2f(*p["at"])
            w, h = p["size"]
            if round(p.get("angle", 0) + fp["rot"]) % 180:
                w, h = h, w
            if p.get("drill"):
                cu -= Pos(px, py, z0) * Cylinder(max(w, h) / 2 + 0.4, bt, align=C)
            else:
                zc = z0 if fp["side"] == "B" else z1 - 0.05
                cu -= Pos(px, py, zc) * Box(w + 0.8, h + 0.8, 0.05, align=C)
    out.append(DPart("School-milled board (FR4, 46 x 38 mm)", "avionics", fr4, FR4, 0.0, 0.8, "board"))
    out.append(DPart("Board copper pour", "avionics", cu, COPPER, 0.6, 0.45, "board"))

    for fp in lay["footprints"]:
        ref, kind, side = fp["ref"], fp["footprint"], fp["side"]
        cx0, cy0, cx1, cy1 = fp["courtyard"]
        fx0, fx1 = cx0, cx1
        fy0, fy1 = -cy1, -cy0
        pads = [p for p in fp["pads"] if not p.get("npth")]
        # tinned pads (top side for THT, own side for SMD)
        for p in pads:
            px, py = _b2f(*p["at"])
            w, h = p["size"]
            if round(p.get("angle", 0) + fp["rot"]) % 180:
                w, h = h, w
            if p.get("drill"):
                pad = Pos(px, py, z1) * Cylinder(max(w, h) / 2, 0.05, align=C) - Pos(px, py, z1) * Cylinder(p["drill"] / 2, 0.05, align=C)
            else:
                zc = z1 if side == "F" else z0 - 0.05
                pad = Pos(px, py, zc) * Box(w, h, 0.05, align=C)
            out.append(DPart(f"{ref} pad", "avionics", pad, SOLDER, 0.8, 0.35))
        val = fp["value"]
        label = f"{ref}: {val}"
        if kind in _SMD_H:  # bottom-side SMD parts: body inside the courtyard, under the board
            h = _SMD_H[kind]
            if h == 0:
                continue
            cxm, cym = (fx0 + fx1) / 2, (fy0 + fy1) / 2
            wx, wy = (fx1 - fx0) - 1.0, (fy1 - fy0) - 1.0
            if kind.startswith("R_") or kind.startswith("C_"):
                long_x = wx > wy
                L, W = (wx, wy) if long_x else (wy, wx)
                L, W = min(L, 3.2), min(W, 1.6)
                body_c = CAP_TAN if kind.startswith("C_") else BLACK
                body = Pos(cxm, cym, z0 - h) * Box(L - 1.0 if long_x else W, W if long_x else L - 1.0, h, align=C)
                out.append(DPart(label, "avionics", body, body_c, 0.0, 0.6, "board"))
                for s in (-1, 1):
                    ex, ey = ((s * (L / 2 - 0.25), 0) if long_x else (0, s * (L / 2 - 0.25)))
                    cap = Pos(cxm + ex, cym + ey, z0 - h) * Box(0.5 if long_x else W, W if long_x else 0.5, h, align=C)
                    out.append(DPart(label, "avionics", cap, SOLDER, 0.85, 0.3, "board"))
            else:
                body = Pos(cxm, cym, z0 - h) * Box(min(wx, 2.9), min(wy, 2.9) if kind.startswith("SOT") else min(wy, 2.7), h, align=C)
                if kind.startswith("D_"):
                    body = Pos(cxm, cym, z0 - h) * Box(min(wx, 1.6), min(wy, 2.7), h, align=C)
                out.append(DPart(label, "avionics", body, BLACK, 0.0, 0.55, "board"))
            continue
        if kind.startswith("CP_Radial"):  # 470 uF can: sleeve, top, under the ESP32 edge
            ccx, ccy = (fx0 + fx1) / 2, (fy0 + fy1) / 2
            out.append(DPart(label, "avionics", Pos(ccx, ccy, z1) * Cylinder(3.15, 7.0, align=C), 0x1F3A93, 0.1, 0.5, "top"))
            out.append(DPart(label, "avionics", Pos(ccx, ccy, z1 + 7.0) * Cylinder(2.9, 0.4, align=C), SILVER, 0.9, 0.3, "top"))
        elif kind.startswith("SW_SPDT"):
            sx0, sx1 = fx0 + 0.6, fx1 - 0.6
            sy0, sy1 = fy0 + 1.1, fy1 - 1.1
            out.append(DPart(label, "avionics", _box(sx0, sx1, sy0, sy1, z1, 3.5), SILVER, 0.85, 0.35, "top"))
            mx = (sx0 + sx1) / 2 + 1.0
            out.append(DPart(label + " lever", "avionics", _box(mx - 0.75, mx + 0.75, sy0 - 2.5, sy0, z1 + 1.0, 1.5), BLACK, 0.0, 0.6, "top"))
        elif kind.startswith("JST_PH"):
            out.append(DPart(label, "avionics", _box(fx0 + 0.3, fx1 - 0.3, fy0 + 0.3, fy1 - 0.3, z1, 6.0), 0xF2EFE6, 0.0, 0.7, "top"))
        elif kind.startswith("LED_D3"):
            lx, ly = (fx0 + fx1) / 2, (fy0 + fy1) / 2
            led = Pos(lx, ly, z1) * Cylinder(1.5, 3.8, align=C) + Pos(lx, ly, z1 + 3.8) * Sphere(1.5)
            led += Pos(lx, ly, z1) * Cylinder(1.9, 1.0, align=C)
            out.append(DPart(label, "avionics", led, 0x2ECC71, 0.0, 0.2, "top", opacity=0.85))
        elif kind.startswith("MotorPads"):
            pass  # pads drawn above; the motor leads solder straight on
    # female headers for the ESP32 (one per inner pad row of U1)
    u1 = next(f for f in lay["footprints"] if f["ref"] == "U1")
    cols = {}
    for p in u1["pads"]:
        cols.setdefault(round(p["at"][0], 2), []).append(-p["at"][1])
    hdr = g["esp32_header_height"]
    for x, ys in cols.items():
        hb = _box(x - 1.27, x + 1.27, min(ys) - 1.27, max(ys) + 1.27, z1, hdr)
        out.append(DPart(f"Female header ({len(ys)}-pin, 8.5 mm)", "avionics", hb, BLACK, 0.0, 0.7, "top"))
    return out


# ------------------------------------------------------------------ modules
def esp32_detail(spec):
    g, d = spec["geometry_mm"], dims(spec)
    em = g["esp32_module"]
    cx, cy = em["centre"]
    sx, sy = em["size"]
    z = d["board_z1"] + g["esp32_header_height"]
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    out = [DPart("ESP32 MiniKit PCB", "avionics", _box(x0, x1, y0, y1, z, 1.0), BLUE_PCB, 0.1, 0.55, "esp32")]
    # WROOM-32: 18 x 25.5 mm, antenna end flush with the +y edge
    wx0, wx1, wy0, wy1 = cx - 9, cx + 9, y1 - 25.5, y1
    zt = z + 1.0
    out.append(DPart("ESP32-WROOM-32 module", "avionics", _box(wx0, wx1, wy0, wy1, zt, 0.8), BLACK_PCB, 0.1, 0.6, "esp32"))
    out.append(DPart("ESP32-WROOM-32 shield can", "avionics", _box(wx0 + 0.6, wx1 - 0.6, wy0 + 0.6, wy0 + 18.0, zt + 0.8, 2.4), SILVER, 0.9, 0.3, "esp32"))
    out.append(DPart("PCB antenna", "avionics", _box(wx0 + 1.0, wx1 - 1.0, wy1 - 5.5, wy1 - 1.0, zt + 0.8, 0.04), 0xC9A227, 0.7, 0.4, "esp32"))
    out.append(DPart("Micro-USB (programming)", "avionics", _box(cx - 3.75, cx + 3.75, y0, y0 + 5.0, zt, 2.5), SILVER, 0.9, 0.3, "esp32"))
    out.append(DPart("CP2104 USB-UART", "avionics", _box(cx - 2, cx + 2, y0 + 7.0, y0 + 11.0, zt, 0.9), BLACK, 0.0, 0.5, "esp32"))
    out.append(DPart("3.3 V regulator", "avionics", _box(cx + 6, cx + 9, y0 + 6.5, y0 + 8.0, zt, 1.1), BLACK, 0.0, 0.5, "esp32"))
    for k in range(2):  # reset / boot buttons
        bx_ = x0 + 2.5 + 4.5 * k
        out.append(DPart("Button", "avionics", _box(bx_, bx_ + 3.0, y0 + 2.0, y0 + 6.0, zt, 1.5), SILVER, 0.8, 0.35, "esp32"))
    return out


def imu_detail(spec):
    g, d = spec["geometry_mm"], dims(spec)
    im = g["imu_module"]
    z = d["board_z1"]
    x0, x1, y0, y1 = im["x"][0] + 0.2, im["x"][1] - 0.2, im["y"][0] + 0.2, im["y"][1] - 0.2
    out = [DPart("GY-521 header spacer", "avionics", _box(x0, x0 + 2.54, y0 + 0.2, y1 - 0.2, z, 1.5), BLACK, 0.0, 0.7, "imu"),
           DPart("GY-521 (MPU-6050) PCB", "avionics", _box(x0, x1, y0, y1, z + 1.5, 1.6), BLUE_PCB, 0.1, 0.55, "imu")]
    mx, my = (x0 + x1) / 2 + 1.0, (y0 + y1) / 2
    out.append(DPart("MPU-6050", "avionics", _box(mx - 2, mx + 2, my - 2, my + 2, z + 3.1, 0.9), BLACK, 0.0, 0.45, "imu"))
    out.append(DPart("GY-521 regulator", "avionics", _box(mx - 1.5, mx + 1.5, y1 - 5, y1 - 2, z + 3.1, 0.9), BLACK, 0.0, 0.5, "imu"))
    return out


def charger_detail(spec):
    g, d = spec["geometry_mm"], dims(spec)
    cm = g["charger_module"]
    z = d["board_z1"]
    x0, x1, y0, y1 = cm["x"][0] + 0.2, cm["x"][1] - 0.2, cm["y"][0] + 0.2, cm["y"][1] - 0.2
    out = [DPart("TP4056 charger PCB", "avionics", _box(x0, x1, y0, y1, z + 0.6, 1.2), BLUE_PCB, 0.1, 0.55, "charger")]
    yc = (y0 + y1) / 2
    out.append(DPart("Micro-USB (charging, faces the nose)", "avionics", _box(x1 - 5.5, x1 + 0.0, yc - 3.75, yc + 3.75, z + 1.8, 2.6), SILVER, 0.9, 0.3, "charger"))
    out.append(DPart("TP4056", "avionics", _box(x0 + 9, x0 + 14, yc - 2.5, yc + 2.5, z + 1.8, 1.5), BLACK, 0.0, 0.5, "charger"))
    out.append(DPart("DW01 protection", "avionics", _box(x0 + 2, x0 + 5, yc - 1.5, yc + 1.5, z + 1.8, 1.1), BLACK, 0.0, 0.5, "charger"))
    for k, col in ((0, RED), (1, 0x2980B9)):
        out.append(DPart("Charge LED", "avionics", _box(x0 + 16 + 2.2 * k, x0 + 17.6 + 2.2 * k, y0 + 1.5, y0 + 2.3, z + 1.8, 0.6), col, 0.0, 0.3, "charger"))
    return out


def battery_detail(spec):
    g, d = spec["geometry_mm"], dims(spec)
    P = spec["parts"]["battery"]
    e = P["envelope_mm"]
    xo, z = P.get("x_offset", 0.0), d["t"] + 0.5
    body = Pos(xo + 1.5, 0, z) * Box(e["x"] - 3.0, e["y"], e["z"], align=C)
    body = fillet(body.edges().filter_by(lambda ed: True), 1.6)
    out = [DPart("1S 600 mAh LiPo", "battery", body, LIPO, 0.3, 0.5, "battery")]
    # tab end: sealed foil band with the leads out of the rear
    band = Pos(xo - e["x"] / 2 + 1.5, 0, z + 1.5) * Box(3.0, e["y"] - 4, e["z"] - 3, align=C)
    out.append(DPart("Cell tabs (sealed)", "battery", band, SILVER, 0.8, 0.35, "battery"))
    label = Pos(xo + 3, 0, z + e["z"] - 0.01) * Box(e["x"] - 14, e["y"] - 6, 0.01, align=C)
    out.append(DPart("Battery label", "battery", label, 0xD4AC0D, 0.0, 0.6, "battery"))
    return out


def autonomy_detail(spec):
    """v3: PMW3901 flow and VL53L1X ToF breakouts on the tail tab (lens / sensor looking down
    through the rim windows), XIAO ESP32-S3 Sense standing behind the nose arch, camera forward."""
    g, d = spec["geometry_mm"], dims(spec)
    au = g.get("autonomy", {})
    if not au.get("fit"):
        return []
    out, t = [], d["t"]
    for key, owner, label, chip in (("flow_module", "flow", "PMW3901 optical-flow breakout", "PMW3901 + lens"),
                                    ("tof_module", "tof", "VL53L1X time-of-flight breakout", "VL53L1X")):
        m = au[key]
        (sx, sy, sz), (cx, cy) = m["size"], m["centre"]
        z_lo, z_hi = t, t + sz   # sits on the tab; chip + lens under the PCB, looking down the window
        out.append(DPart(label, "autonomy", _box(cx - sx / 2 + 0.1, cx + sx / 2 - 0.1, cy - sy / 2 + 0.1, cy + sy / 2 - 0.1,
                                                 z_hi - 1.1, 1.0), 0x6C3483 if owner == "flow" else BLUE_PCB, 0.1, 0.55, owner))
        if owner == "flow":
            out.append(DPart(chip, "autonomy", Pos(cx, cy, z_lo + 0.1) * Cylinder(2.6, z_hi - 1.1 - z_lo - 0.1, align=C), BLACK, 0.0, 0.3, owner))
            out.append(DPart("Flow lens glass", "autonomy", Pos(cx, cy, z_lo + 0.1) * Cylinder(1.4, 0.3, align=C), 0x5DADE2, 0.3, 0.1, owner, opacity=0.8))
        else:
            out.append(DPart(chip, "autonomy", _box(cx - 2.4, cx + 2.4, cy - 1.2, cy + 1.2, z_hi - 1.1 - 1.3, 1.3), BLACK, 0.0, 0.4, owner))
    cam = au["camera"]
    cy_, cz_, cdep = cam["size"]
    x1 = au["arch"]["x0"]
    x0 = x1 - cdep
    yc, z0 = cam["y_centre"], cam["z0"]
    ylo, yhi, zlo, zhi = yc - cy_ / 2 + 0.1, yc + cy_ / 2 - 0.1, z0 + 0.1, z0 + cz_ - 0.1
    zc = z0 + cz_ / 2
    pcb = lambda xa, xb: Pos((xa + xb) / 2, yc, zlo) * Box(xb - xa, yhi - ylo, zhi - zlo, align=C)
    out.append(DPart("XIAO ESP32-S3 (main board)", "autonomy", pcb(x0 + 0.2, x0 + 1.4), BLUE_PCB, 0.1, 0.55, "camera"))
    out.append(DPart("ESP32-S3 module shield", "autonomy", Pos(x0 + 2.4, yc, zlo + 1.0) * Box(2.0, 12.5, 13.0, align=C), SILVER, 0.9, 0.3, "camera"))
    out.append(DPart("USB-C (programming)", "autonomy", Pos(x0 + 2.4, yc, zhi - 3.3) * Box(2.0, 8.9, 3.2, align=C), SILVER, 0.9, 0.3, "camera"))
    out.append(DPart("Sense expansion board", "autonomy", pcb(x0 + 6.0, x0 + 7.2), BLACK_PCB, 0.1, 0.55, "camera"))
    out.append(DPart("OV2640 camera module", "autonomy", Pos(x0 + 9.2, yc, zc + 2.5 - 4.25) * Box(4.0, 8.5, 8.5, align=C), BLACK, 0.0, 0.5, "camera"))
    out.append(DPart("Camera lens", "autonomy", Pos(x0 + 11.2, yc, zc + 2.5) * Rot(0, 90, 0) * Cylinder(3.2, x1 - 0.1 - x0 - 11.2, align=C),
                     DARK, 0.2, 0.25, "camera"))
    out.append(DPart("Camera lens glass", "autonomy", Pos(x1 - 0.5, yc, zc + 2.5) * Rot(0, 90, 0) * Cylinder(2.0, 0.35, align=C),
                     0x5DADE2, 0.3, 0.1, "camera", opacity=0.8))
    return out


# ------------------------------------------------------------------ assembly
def build_detail(spec=None, with_guard=True):
    spec = spec or load_spec()
    g = spec["geometry_mm"]
    parts = [DPart("Frame (one-piece PETG print)", "frame", frame(spec), PETG, 0.0, 0.55)]
    for i, (x, y) in enumerate(motor_xy(spec)):
        parts += motor_detail(spec, i, x, y)
        parts += prop_detail(spec, i, x, y)
    parts += board_detail(spec) + esp32_detail(spec) + imu_detail(spec) + charger_detail(spec) + battery_detail(spec)
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

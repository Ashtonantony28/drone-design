"""NF-3D Lite: one-piece PLA micro frame in build123d.

Body frame FLU as in v1 (x forward, y left, z up); origin at the frame centre on
the bottom face of the centre plate. Motors in neuroflight order:
  0 front-left (+a,+a)  1 front-right (+a,-a)  2 rear-right (-a,-a)  3 rear-left (-a,+a)

Layout (side view): the battery lies on the centre plate, the school-made board
sits on four tall bosses above it, and the ESP32 module plugs in on top. The
motors stand in press-fit tubes at the arm tips, high enough that the props spin
above the electronics. The whole underside is one flat plane: it prints with no
supports and it is what the drone lands on.

Every number comes from lite/spec.yaml.
"""
from __future__ import annotations

import math
import os

import yaml
from build123d import Align, Axis, Box, Cylinder, Kind, Pos, Rot, extrude, offset

from mech.frame import Component  # same dataclass, so the v1 check helpers apply

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC_PATH = os.path.join(HERE, "spec.yaml")
MOTOR_SIGNS = [(1, 1), (1, -1), (-1, -1), (-1, 1)]
C = (Align.CENTER, Align.CENTER, Align.MIN)


def load_spec(path: str = SPEC_PATH) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def motor_xy(spec):
    a = spec["geometry_mm"]["arm_xy"]
    return [(sx * a, sy * a) for sx, sy in MOTOR_SIGNS]


def dims(spec):
    g = spec["geometry_mm"]
    d = {"t": g["plate_thickness"], "tube_ro": g["motor_bore"] / 2 + g["motor_tube_wall"],
         "tube_ri": g["motor_bore"] / 2, "prop_r": g["prop_diameter"] / 2}
    d["motor_z1"] = g["motor_z0"] + spec["parts"]["motor"]["envelope_mm"]["height"]
    d["prop_z0"] = d["motor_z1"] + g["prop_gap_above_motor"]
    d["prop_z1"] = d["prop_z0"] + g["prop_band"]
    d["board_z0"] = d["t"] + g["board_boss_height"]
    d["board_z1"] = d["board_z0"] + g["board_size"][2]
    d["avionics_z1"] = d["board_z1"] + g["avionics_height"]
    return d


def frame(spec) -> "Part":
    """The printed frame: centre plate, arms, motor tubes, legs, board bosses."""
    g, d = spec["geometry_mm"], dims(spec)
    t = d["t"]
    z_lo, z_hi = g["motor_tube_z"]
    # 2-D outline of the flat parts (plate + arms), filleted at the arm roots
    flat = Box(g["centre_x"], g["centre_y"], g["arm_height"], align=C)
    for (x, y) in motor_xy(spec):
        L = math.hypot(x, y)
        flat += Rot(0, 0, math.degrees(math.atan2(y, x))) * Box(L, g["arm_width"], g["arm_height"],
                                                             align=(Align.MIN, Align.CENTER, Align.MIN))
    outline = flat.faces().sort_by(Axis.Z)[0]
    r = g["arm_root_fillet"]
    outline = offset(offset(outline, r, kind=Kind.ARC), -r, kind=Kind.ARC)
    body = extrude(outline, amount=g["arm_height"], dir=(0, 0, 1))
    # thin the centre back to plate thickness (arms stay tall), leaving side rims
    # open at the front and back so the 48 mm battery slides in lengthwise; the side rims stay
    pocket = Box(g["centre_x"] + 2, g["centre_y"] - 6, g["arm_height"] - t, align=C)
    body -= Pos(0, 0, t) * pocket
    for (x, y) in motor_xy(spec):
        body += Pos(x, y, z_lo) * Cylinder(d["tube_ro"], z_hi - z_lo, align=C)
    # motor bores: through the tube top down to a 1 mm ledge at motor_z0, a smaller
    # hole below lets the motor be pushed out and the leads exit
    for (x, y) in motor_xy(spec):
        body -= Pos(x, y, g["motor_z0"]) * Cylinder(d["tube_ri"], z_hi - g["motor_z0"] + 1, align=C)
        body -= Pos(x, y, z_lo - 1) * Cylinder(d["tube_ri"] - 1.0, g["motor_z0"] - z_lo + 2, align=C)
    # board bosses with M2 pilot holes
    hx, hy = g["board_holes"]
    for sx in (-1, 1):
        for sy in (-1, 1):
            body += Pos(sx * hx, sy * hy, t) * Cylinder(g["board_boss_od"] / 2, g["board_boss_height"], align=C)
            body -= Pos(sx * hx, sy * hy, 0) * Cylinder(g["board_hole"] / 2, t + g["board_boss_height"], align=C)
    # battery strap slots and a lightening/wire window
    sw, sh = g["strap_slot"]
    for sx in (-1, 1):
        body -= Pos(sx * 12.0, 0, 0) * Box(sh, sw, t, align=C)
    body -= Box(14, 18, t, align=C)
    return body


# ---------------------------------------------------------------- bought parts (envelopes)
def motor(spec, x, y):
    g, e = spec["geometry_mm"], spec["parts"]["motor"]["envelope_mm"]
    return Pos(x, y, g["motor_z0"]) * Cylinder(e["diameter"] / 2 - 0.05, e["height"], align=C)


def prop_disk(spec, x, y):
    d = dims(spec)
    return Pos(x, y, d["prop_z0"]) * Cylinder(d["prop_r"], d["prop_z1"] - d["prop_z0"], align=C)


def build_assembly(spec=None) -> list[Component]:
    spec = spec or load_spec()
    g, d, P = spec["geometry_mm"], dims(spec), spec["parts"]
    comps = [Component("frame", frame(spec), material="pla_print")]
    for i, (x, y) in enumerate(motor_xy(spec)):
        comps.append(Component(f"motor_{i}", motor(spec, x, y), mass_g=P["motor"]["mass_g"],
                               group="propulsion", touches={"frame"}))
    bx, by, bt = g["board_size"]
    board = Box(bx, by, bt, align=C)
    hx, hy = g["board_holes"]
    for sx in (-1, 1):
        for sy in (-1, 1):
            board -= Pos(sx * hx, sy * hy, 0) * Cylinder(1.1, bt, align=C)
    # bottom-side SMD parts (lite/ecad/MILLING.md) as an envelope under the board, so the
    # battery clearance check sees them
    ux, uy = 2 * (hx - g["board_boss_od"] / 2 - 0.5), by - 6  # stops short of the bosses
    board += Pos(0, 0, -g["board_underside_parts"]) * Box(ux, uy, g["board_underside_parts"], align=C)
    comps.append(Component("board", Pos(0, 0, d["board_z0"]) * board, mass_g=P["board"]["mass_g"] + P["charger"]["mass_g"],
                           group="avionics", touches={"frame"}))
    # plugged-in modules: ESP32 D1 mini over the board, IMU breakout beside it (envelopes)
    esp = Box(39.0, 31.0, g["avionics_height"], align=C)
    comps.append(Component("esp32", Pos(0, 0, d["board_z1"]) * esp, mass_g=P["esp32"]["mass_g"] + P["imu"]["mass_g"],
                           group="avionics", touches={"board"}))
    e = P["battery"]["envelope_mm"]
    comps.append(Component("battery", Pos(0, 0, d["t"] + 0.5) * Box(e["x"], e["y"], e["z"], align=C),
                           mass_g=P["battery"]["mass_g"], group="battery", touches={"frame"}))
    return comps


def lumped_masses(spec):
    d = dims(spec)
    zp = (d["prop_z0"] + d["prop_z1"]) / 2
    out = [(f"prop_{i}", spec["parts"]["prop"]["mass_g"], (x, y, zp)) for i, (x, y) in enumerate(motor_xy(spec))]
    out.append(("hardware", spec["parts"]["hardware"]["mass_g"], (0.0, 0.0, 2.0)))
    out.append(("wiring", spec["parts"]["wiring"]["mass_g"], (0.0, 0.0, 4.0)))
    return out

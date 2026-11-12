"""Parametric NF-3D airframe in build123d.

Coordinate system (matches neuroflight-fc docs/INTERFACES.md body frame FLU):
x forward, y left, z up. Origin is the frame centre on the *bottom face* of the
bottom plate. Motor i sits at the INTERFACES.md §2 position:
  0 front-left (+a,+a)  1 front-right (+a,-a)  2 rear-right (-a,-a)  3 rear-left (-a,+a)

Every dimension comes from spec/vehicle.yaml. Functions return build123d Parts
positioned in the assembly frame, so checks can run directly on them.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import yaml
from build123d import (Align, Axis, Box, BuildSketch, Cylinder, Kind, Locations, Part,
                       Plane, Pos, Rectangle, Rot, Torus, extrude, fillet, offset,
                       revolve)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SPEC_PATH = os.path.join(ROOT, "spec", "vehicle.yaml")

MOTOR_SIGNS = [(1, 1), (1, -1), (-1, -1), (-1, 1)]  # INTERFACES.md §2 order


def load_spec(path: str = SPEC_PATH) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def motor_xy(spec: dict) -> list[tuple[float, float]]:
    a = spec["geometry_mm"]["arm_xy"]
    return [(sx * a, sy * a) for sx, sy in MOTOR_SIGNS]


@dataclass
class Component:
    """One solid in the assembly with what the mass model needs."""
    name: str
    part: Part
    material: str | None = None   # key into spec['materials'] (density from volume)
    mass_g: float | None = None   # bought part: catalogue mass spread uniformly over the envelope
    group: str = "frame"
    touches: set = field(default_factory=set)  # names it is allowed to touch (mounted against)


# --------------------------------------------------------------------------- derived dims
def dims(spec: dict) -> dict:
    g = spec["geometry_mm"]
    d = {}
    d["prop_r"] = g["prop_diameter"] / 2
    d["duct_ri"] = d["prop_r"] + g["prop_tip_gap"]
    d["duct_ro"] = d["duct_ri"] + g["duct_wall"]
    d["t_bot"] = g["bottom_plate_thickness"]
    d["t_top"] = g["top_plate_thickness"]
    d["z_top_plate"] = d["t_bot"] + g["standoff_height"]
    m = spec["parts"]["motor"]["envelope_mm"]
    d["motor_r"] = m["diameter"] / 2
    d["motor_h"] = m["height"]
    # prop: hub sits on top of the bell (bell = envelope height minus the 3 mm shaft
    # stub); the spinning prop occupies a `prop_band` thick slab
    d["bell_h"] = d["motor_h"] - 3.0
    d["prop_z0"] = d["t_bot"] + d["bell_h"]
    d["prop_z1"] = d["prop_z0"] + g["prop_band"]
    # duct: sits on the plate ring and ends above the blade band
    d["duct_z0"] = d["t_bot"]
    d["duct_z1"] = d["duct_z0"] + g["duct_height"]
    d["ring_ri"] = d["duct_ri"] - 0.0
    d["ring_ro"] = d["duct_ro"] + g["ring_extension"]
    return d


# --------------------------------------------------------------------------- frame parts
def bottom_plate(spec: dict) -> Part:
    """Carbon unibody: centre square, 4 arms, motor pads, duct support rings."""
    g, d = spec["geometry_mm"], dims(spec)
    t = d["t_bot"]
    body = Box(g["centre_body"], g["centre_body"], t, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for (x, y) in motor_xy(spec):
        L = math.hypot(x, y)
        ang = math.degrees(math.atan2(y, x))
        arm = Box(L, g["arm_width"], t, align=(Align.MIN, Align.CENTER, Align.MIN))
        body += Rot(0, 0, ang) * arm
        body += Pos(x, y, 0) * Cylinder(11.0, t, align=(Align.CENTER, Align.CENTER, Align.MIN))
        ring = (Cylinder(d["ring_ro"], t, align=(Align.CENTER, Align.CENTER, Align.MIN))
                - Cylinder(d["ring_ri"], t, align=(Align.CENTER, Align.CENTER, Align.MIN)))
        body += Pos(x, y, 0) * ring
        # three spokes pad -> ring (120 deg apart, one along the arm direction outward)
        for k in range(3):
            a_sp = ang + 60 + 120 * k
            spoke = Box(d["ring_ri"] + 1.0, g["spoke_width"], t, align=(Align.MIN, Align.CENTER, Align.MIN))
            body += Pos(x, y, 0) * Rot(0, 0, a_sp) * spoke
    # bridges between neighbouring rings: front, right, rear, left sides. They
    # close the perimeter so a hit on one duct is shared by the whole plate.
    a = g["arm_xy"]
    gap = 2 * a - 2 * d["ring_ro"] + 4.0  # overlap 2 mm into each ring
    for (cx, cy, rot) in ((a, 0, 90), (0, -a, 0), (-a, 0, 90), (0, a, 0)):
        bridge = Box(gap, g["side_bridge_width"], t, align=(Align.CENTER, Align.CENTER, Align.MIN))
        # place on the outer edge of the rings, tangent line between ring centres
        body += Pos(cx, cy, 0) * Rot(0, 0, rot) * bridge
    # round every vertical edge of the outline: removes the sharp re-entrant
    # corners at the arm roots and bridge joints where the FEA peaks sat
    # (morphological closing of the 2-D outline: offset out, then back in)
    r = g["plate_fillet"]
    outline = body.faces().sort_by(Axis.Z)[0]
    closed = offset(offset(outline, r, kind=Kind.ARC), -r, kind=Kind.ARC)
    body = extrude(closed, amount=t, dir=(0, 0, 1))
    # holes: stack, motor screws, shaft clearance, lightening cut in the centre
    s = g["stack_spacing"] / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            body -= Pos(sx * s, sy * s, 0) * Cylinder(g["stack_hole"] / 2, t, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for (x, y) in motor_xy(spec):
        body -= Pos(x, y, 0) * Cylinder(g["motor_shaft_clearance"] / 2, t, align=(Align.CENTER, Align.CENTER, Align.MIN))
        r = g["motor_mount_circle"] / 2
        for k in range(4):
            a_h = math.radians(45 + 90 * k)
            body -= Pos(x + r * math.cos(a_h), y + r * math.sin(a_h), 0) * Cylinder(
                g["motor_mount_hole"] / 2, t, align=(Align.CENTER, Align.CENTER, Align.MIN))
    body -= Box(16, 16, t, align=(Align.CENTER, Align.CENTER, Align.MIN))  # wire pass + flow-sensor window
    return body


def top_plate(spec: dict) -> Part:
    g, d = spec["geometry_mm"], dims(spec)
    side = g["stack_spacing"] + 9.0
    p = Box(side, side, d["t_top"], align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = fillet(p.edges().filter_by(Axis.Z), 3.0)
    s = g["stack_spacing"] / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            p -= Pos(sx * s, sy * s, 0) * Cylinder(g["stack_hole"] / 2, d["t_top"], align=(Align.CENTER, Align.CENTER, Align.MIN))
    for sy in (-1, 1):  # battery strap slots
        p -= Pos(0, sy * (side / 2 - 3.5), 0) * Box(20, 2.5, d["t_top"], align=(Align.CENTER, Align.CENTER, Align.MIN))
    return Pos(0, 0, d["z_top_plate"]) * p


def standoff(spec: dict, x: float, y: float) -> Part:
    g, d = spec["geometry_mm"], dims(spec)
    h = g["standoff_height"]
    # M3 aluminium screw/standoff column; the boards ride on it through grommets
    return Pos(x, y, d["t_bot"]) * Cylinder(1.5, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def duct(spec: dict, x: float, y: float) -> Part:
    """Printed duct: straight shroud with a rounded (bell-mouth) inlet lip on top.

    Built by revolving its cross-section: the wall rectangle plus a quarter-round
    lip that flares outward at the top edge.
    """
    g, d = spec["geometry_mm"], dims(spec)
    h, lr, ri, ro = g["duct_height"], g["duct_lip_radius"], d["duct_ri"], d["duct_ro"]
    with BuildSketch(Plane.XZ) as sk:
        with Locations((ri + (ro - ri) / 2, h / 2)):
            Rectangle(ro - ri, h)
        with Locations((ro, h - lr)):
            Rectangle(lr, lr, align=(Align.MIN, Align.MIN))
    prof = sk.sketch
    solid = revolve(prof, axis=Axis.Z)
    # round the outer top corner of the lip into a quarter circle
    cut = Pos(0, 0, h - lr) * (Cylinder(ro + lr + 1, lr, align=(Align.CENTER, Align.CENTER, Align.MIN))
                               - Cylinder(ro + lr, lr, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    solid -= cut
    torus_cut = Pos(0, 0, h - lr) * Torus(ro, lr)
    corner = Pos(0, 0, h - lr) * (Cylinder(ro + lr, lr, align=(Align.CENTER, Align.CENTER, Align.MIN))
                                  - Cylinder(ro, lr, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    solid -= (corner - torus_cut)
    return Pos(x, y, d["duct_z0"]) * solid


def foot(spec: dict, x: float, y: float) -> Part:
    """TPU landing foot under the outer edge of each duct ring."""
    d = dims(spec)
    r_pos = d["ring_ri"] + (d["ring_ro"] - d["ring_ri"]) / 2
    ang = math.atan2(y, x)
    fx, fy = x + r_pos * math.cos(ang), y + r_pos * math.sin(ang)
    return Pos(fx, fy, -8.0) * Cylinder(3.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN))


# --------------------------------------------------------------------------- bought parts (envelopes)
def motor(spec: dict, x: float, y: float) -> Part:
    d = dims(spec)
    return Pos(x, y, d["t_bot"]) * Cylinder(d["motor_r"], d["bell_h"], align=(Align.CENTER, Align.CENTER, Align.MIN))


def prop_disk(spec: dict, x: float, y: float) -> Part:
    """Swept volume of the spinning propeller (used for clearance, not as a solid)."""
    d = dims(spec)
    return Pos(x, y, d["prop_z0"]) * Cylinder(d["prop_r"], d["prop_z1"] - d["prop_z0"], align=(Align.CENTER, Align.CENTER, Align.MIN))


def box_env(spec: dict, key: str, z0: float, xy=(0.0, 0.0), stack_holes: bool = False) -> Part:
    e = spec["parts"][key]["envelope_mm"]
    b = Box(e["x"], e["y"], e["z"], align=(Align.CENTER, Align.CENTER, Align.MIN))
    if stack_holes:  # 30.5 mm boards: 4 mm holes for the grommets
        s = spec["geometry_mm"]["stack_spacing"] / 2
        for sx in (-1, 1):
            for sy in (-1, 1):
                b -= Pos(sx * s, sy * s, 0) * Cylinder(2.0, e["z"], align=(Align.CENTER, Align.CENTER, Align.MIN))
    return Pos(xy[0], xy[1], z0) * b


# --------------------------------------------------------------------------- assembly
def build_assembly(spec: dict | None = None) -> list[Component]:
    spec = spec or load_spec()
    g, d = spec["geometry_mm"], dims(spec)
    comps: list[Component] = []
    s = g["stack_spacing"] / 2
    stack_names = []
    comps.append(Component("bottom_plate", bottom_plate(spec), material="carbon_3k_quasi_iso"))
    comps.append(Component("top_plate", top_plate(spec), material="carbon_3k_quasi_iso"))
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, -1), (-1, 1)]):
        n = f"standoff_{i}"
        stack_names.append(n)
        comps.append(Component(n, standoff(spec, sx * s, sy * s), material="aluminium_7075",
                               touches={"bottom_plate", "top_plate"}))
    for i, (x, y) in enumerate(motor_xy(spec)):
        comps.append(Component(f"duct_{i}", duct(spec, x, y), material="pa12_cf_print", touches={"bottom_plate"}))
        comps.append(Component(f"foot_{i}", foot(spec, x, y), material=None, mass_g=0.4, touches={"bottom_plate"}))
        comps.append(Component(f"motor_{i}", motor(spec, x, y), mass_g=spec["parts"]["motor"]["mass_g"],
                               group="propulsion", touches={"bottom_plate"}))
    # stack: ESC on 4 mm grommets, FC 4 mm above the ESC (grommet + nylon nuts)
    z_esc = d["t_bot"] + 4.0
    z_fc = z_esc + spec["parts"]["esc_4in1"]["envelope_mm"]["z"] + 4.0
    comps.append(Component("esc_4in1", box_env(spec, "esc_4in1", z_esc, stack_holes=True), mass_g=spec["parts"]["esc_4in1"]["mass_g"],
                           group="avionics", touches=set(stack_names)))
    comps.append(Component("fc", box_env(spec, "fc", z_fc, stack_holes=True), mass_g=spec["parts"]["fc"]["mass_g"],
                           group="avionics", touches=set(stack_names)))
    # receiver sits on top of the FC at the rear, under the top plate
    rx_z = z_fc + spec["parts"]["fc"]["envelope_mm"]["z"] + 0.5
    comps.append(Component("receiver", box_env(spec, "receiver", rx_z, (-8.0, 0.0)),
                           mass_g=spec["parts"]["receiver"]["mass_g"], group="avionics",
                           touches={"fc"}))  # foam-taped to the FC
    # flow + ToF under the bottom plate, looking down through the centre window
    fe = spec["parts"]["flow_tof"]["envelope_mm"]
    comps.append(Component("flow_tof", box_env(spec, "flow_tof", -fe["z"]), mass_g=spec["parts"]["flow_tof"]["mass_g"],
                           group="avionics", touches={"bottom_plate"}))
    # battery strapped on the top plate, centred so CoG is on the thrust centre
    comps.append(Component("battery", box_env(spec, "battery", d["z_top_plate"] + d["t_top"]),
                           mass_g=spec["parts"]["battery"]["mass_g"], group="battery", touches={"top_plate"}))
    return comps


def lumped_masses(spec: dict) -> list[tuple[str, float, tuple[float, float, float]]]:
    """Point masses without a solid: props (at the prop plane), hardware, wiring."""
    d = dims(spec)
    out = []
    zp = (d["prop_z0"] + d["prop_z1"]) / 2
    for i, (x, y) in enumerate(motor_xy(spec)):
        out.append((f"prop_{i}", spec["parts"]["prop"]["mass_g"], (x, y, zp)))
    out.append(("hardware", spec["parts"]["hardware"]["mass_g"], (0.0, 0.0, d["t_bot"] + 8.0)))
    out.append(("wiring", spec["parts"]["wiring"]["mass_g"], (0.0, 0.0, d["t_bot"] + 6.0)))
    return out

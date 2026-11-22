"""NF-3D Lite: one-piece printed (PETG, v2) micro frame in build123d.

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
from build123d import Align, Axis, Box, Cone, Cylinder, Kind, Plane, Polygon, Pos, Rot, extrude, offset, revolve

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


def _arm(x, y, w_root, w_tip, h):
    """Tapered arm from the frame centre to a motor axis (a trapezoid prism)."""
    L = math.hypot(x, y)
    pts = [(0, -w_root / 2), (L, -w_tip / 2), (L, w_tip / 2), (0, w_root / 2)]
    return Rot(0, 0, math.degrees(math.atan2(y, x))) * extrude(Polygon(*pts, align=None), amount=h)


def _motor_mount(spec, x, y):
    """v2 motor mount: (solid to add, solid to cut).

    z 0 .. pedestal top: thin-walled pedestal with three diamond vent windows (60-degree
    flanks, so they print without supports) that let air reach the motor's base;
    a 60-degree internal cone up to a 1 mm ledge the motor sits on (lead hole through it);
    then a grip section that is slit on the outboard side, with a V-groove for a zip tie
    that clamps the tube onto the motor. Clamping, not a press fit, so heat creep in the
    plastic cannot loosen the motor.
    """
    g, d = spec["geometry_mm"], dims(spec)
    z_lo, z_hi = g["motor_tube_z"]
    z0 = g["motor_z0"]
    ro, ri = d["tube_ro"], d["tube_ri"]
    rp = ro - g["motor_pedestal_wall"]          # pedestal bore
    r_lead = ri - 1.0                            # lead hole through the ledge
    ledge0 = z0 - 1.0
    cone_h = (rp - r_lead) * math.tan(math.radians(60))
    cone_z0 = ledge0 - cone_h
    add = Pos(x, y, z_lo) * Cylinder(ro, z_hi - z_lo, align=C)
    cut = Pos(x, y, z0) * Cylinder(ri, z_hi - z0 + 1, align=C)                       # motor bore
    cut += Pos(x, y, ledge0 - 0.01) * Cylinder(r_lead, 1.02, align=C)                 # lead hole
    cut += Pos(x, y, cone_z0) * Cone(rp, r_lead, cone_h, align=C)                    # cone under the ledge
    cut += Pos(x, y, z_lo - 1) * Cylinder(rp, cone_z0 - z_lo + 1.01, align=C)          # pedestal bore
    # angles: the arm leaves towards the centre; windows and slit point elsewhere
    a_in = math.degrees(math.atan2(-y, -x))
    win_h = min(7.0, cone_z0 - z_lo - 1.5)
    win_w = win_h / math.tan(math.radians(60))
    zc = z_lo + 1.0 + win_h / 2
    # window prism: its sketch plane faces radially outward (normal +x), vertical diagonal on z
    radial = Plane(origin=(0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    window = extrude(radial * Polygon((0, -win_h / 2), (win_w / 2, 0), (0, win_h / 2), (-win_w / 2, 0), align=None),
                     amount=ro + 1)
    for k in (90, 180, 270):
        cut += Pos(x, y, zc) * Rot(0, 0, a_in + k) * window
    # clamp slit on the outboard side, from the top down to 2 mm above the ledge
    sl = g["motor_slit_width"]
    slit_z0 = z0 + 2.0
    cut += Pos(x, y, slit_z0) * Rot(0, 0, a_in + 180) * Box(ro + 1, sl, z_hi - slit_z0 + 1,
                                                            align=(Align.MIN, Align.CENTER, Align.MIN))
    # zip-tie V-groove (50-degree flanks, 0.6 mm deep) half-way up the grip
    zg = z0 + g["motor_grip_length"] / 2
    groove = revolve(Plane.XZ * Polygon((ro - 0.6, 0), (ro + 1.0, -1.9), (ro + 1.0, 1.9), align=None), axis=Axis.Z)
    cut += Pos(x, y, zg) * groove
    return add, cut


def frame(spec) -> "Part":
    """The printed frame (v2): centre plate, tapered arms, clamped motor mounts, bosses."""
    g, d = spec["geometry_mm"], dims(spec)
    t = d["t"]
    h = g["arm_height"]
    # 2-D outline of the flat parts (plate + arms), filleted at the arm roots
    flat = Box(g["centre_x"], g["centre_y"], h, align=C)
    for (x, y) in motor_xy(spec):
        flat += _arm(x, y, g["arm_root_width"], g["arm_tip_width"], h)
    outline = flat.faces().sort_by(Axis.Z)[0]
    r = g["arm_root_fillet"]
    outline = offset(offset(outline, r, kind=Kind.ARC), -r, kind=Kind.ARC)
    body = extrude(outline, amount=h, dir=(0, 0, 1))
    # thin the centre back to plate thickness (arms stay tall), leaving side rims;
    # open front and back so the 48 mm battery slides in lengthwise
    pocket = Box(g["centre_x"] + 2, g["centre_y"] - 6, h - t, align=C)
    body -= Pos(0, 0, t) * pocket
    cuts = []
    for (x, y) in motor_xy(spec):
        add, cut = _motor_mount(spec, x, y)
        body += add
        cuts.append(cut)
    # board bosses with M2 pilot holes
    hx, hy = g["board_holes"]
    for sx in (-1, 1):
        for sy in (-1, 1):
            body += Pos(sx * hx, sy * hy, t) * Cylinder(g["board_boss_od"] / 2, g["board_boss_height"], align=C)
            cuts.append(Pos(sx * hx, sy * hy, 0) * Cylinder(g["board_hole"] / 2, t + g["board_boss_height"], align=C))
    # v2 rear battery stops (the battery slides in from the front)
    bs = g["battery_stop"]
    for yc in bs["y_centres"]:
        body += Pos((bs["x"][0] + bs["x"][1]) / 2, yc, t) * Box(bs["x"][1] - bs["x"][0], bs["width"], bs["height"], align=C)
    # battery strap slots and the lightening/wire window
    sw, sh = g["strap_slot"]
    for sx in (-1, 1):
        cuts.append(Pos(sx * 12.0, 0, 0) * Box(sh, sw, t, align=C))
    for cx, cy, w, hh in g["plate_windows"]:
        cuts.append(Pos(cx, cy, 0) * Box(w, hh, t, align=C))
    au = g.get("autonomy", {})
    if au.get("fit"):
        body, more = _autonomy_mounts(spec, body)
        cuts += more
    for c in cuts:
        body -= c
    return body


def _autonomy_mounts(spec, body):
    """v3: tail tab carrying the flow + ToF sensors (looking down through windows), nose tab with
    a gothic arch the battery slides through and the camera board is taped behind."""
    g, d = spec["geometry_mm"], dims(spec)
    au, t = g["autonomy"], d["t"]
    cuts = []
    tab = au["tail_tab"]
    x0, x1 = tab["x"]
    body += Pos((x0 + x1) / 2, 0, 0) * Box(x1 - x0, 2 * tab["half_width"], t, align=C)
    nt = au["nose_tab"]
    n0, n1 = nt["x"]
    for sy in (-1, 1):  # two side bars; the battery slides in between, under the arch
        body += Pos((n0 + n1) / 2, sy * (nt["half_width"] - nt["bar"] / 2), 0) * Box(n1 - n0, nt["bar"], t, align=C)
    body += Pos(au["arch"]["x0"] + au["arch"]["thickness"] / 2, 0, 0) * Box(au["arch"]["thickness"] + 2.0, 2 * nt["half_width"], t, align=C)
    # each sensor breakout sits on a rim round a window (light, and the lens / ToF apertures see the floor)
    for m in (au["flow_module"], au["tof_module"]):
        sx, sy = m["size"][0] - 2 * tab["rim"], m["size"][1] - 2 * tab["rim"]
        cuts.append(Pos(m["centre"][0], m["centre"][1], 0) * Box(sx, sy, t, align=C))
    a = au["arch"]
    hw, H, ow, wall = a["half_width"], a["height"], a["opening_half_width"], a["opening_wall"]
    apex = wall + ow * math.tan(math.radians(a["flank_deg"]))
    outer = Polygon((-hw, 0), (hw, 0), (hw, H), (-hw, H), align=None)
    opening = Polygon((-ow, -0.1), (ow, -0.1), (ow, wall), (0, apex), (-ow, wall), align=None)
    pl = Plane(origin=(a["x0"], 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    arch = extrude(pl * (outer - opening), amount=a["thickness"])
    body += arch
    # zip-tie slots either side of the camera board, and two gussets behind the arch posts
    sw, sh = a["tie_slot"]
    cam = au["camera"]
    yc, cw = cam["y_centre"], cam["size"][0]
    zc = cam["z0"] + cam["size"][1] / 2
    for ys in (yc - cw / 2 - 1.5, yc + cw / 2 + 1.5):
        cuts.append(Pos(a["x0"] + a["thickness"] / 2, ys, zc) * Box(a["thickness"] + 0.2, sw, sh, align=(Align.CENTER,) * 3))
    gx = Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
    for yg in (hw - 1.5, -hw):
        tri = Polygon((a["x0"], 0), (a["x0"] - 8.0, 0), (a["x0"], 12.0), align=None)
        body += Pos(0, yg, 0) * extrude(gx * tri, amount=-1.5)
    return body, cuts


def guard(spec) -> "Part":
    """Optional clip-on prop guard (v2): one flat printed piece.

    Four 270-degree rings (outside each prop's tip circle, the inner quarter facing the
    centre left out) whose ends meet their neighbours', each held by two spokes to a
    collar that slips over the top of its motor tube above the zip-tie groove. It sits
    below the props, so it fends off walls and furniture rather than shrouding the prop.
    Prints flat, no supports.
    """
    g, d = spec["geometry_mm"], dims(spec)
    G = g["guard"]
    z_hi = g["motor_tube_z"][1]
    zb = z_hi - G["height"]
    ro_t = d["tube_ro"]
    rc_i, rc_o = ro_t + G["collar_clearance"], ro_t + G["collar_clearance"] + G["collar_wall"]
    r_in, r_out = G["ring_radius"], G["ring_radius"] + G["ring_width"]
    part = None
    for (x, y) in motor_xy(spec):
        a_out = math.atan2(y, x)                                      # outboard direction
        ring = Cylinder(r_out, G["height"], align=C) - Cylinder(r_in, G["height"], align=C)
        # remove the inner quarter (a 90-degree wedge facing the frame centre)
        a_in = a_out + math.pi
        R = r_out + 2
        wedge = extrude(Polygon((0, 0), (R * math.cos(a_in - math.pi / 4) * 1.5, R * math.sin(a_in - math.pi / 4) * 1.5),
                                (R * math.cos(a_in) * 2.2, R * math.sin(a_in) * 2.2),
                                (R * math.cos(a_in + math.pi / 4) * 1.5, R * math.sin(a_in + math.pi / 4) * 1.5), align=None),
                        amount=G["height"])
        ring -= wedge
        piece = ring + (Cylinder(rc_o, G["height"], align=C) - Cylinder(rc_i, G["height"], align=C))
        for k in (-1, 1):
            a = a_out + k * math.radians(G["spoke_angle_deg"])
            L = r_in - rc_o + 0.6
            piece += Rot(0, 0, math.degrees(a)) * Pos(rc_o - 0.3, 0, 0) * Box(L, G["spoke_width"], G["height"],
                                                                           align=(Align.MIN, Align.CENTER, Align.MIN))
        piece = Pos(x, y, zb) * piece
        part = piece if part is None else part + piece
    au = spec["geometry_mm"].get("autonomy", {})
    if au.get("fit"):
        # v3: the front rings stop short of the camera and the nose arch (which guards the front)
        a = au["arch"]
        kx0 = a["x0"] - au["camera"]["size"][2] - 2.0
        kx1 = a["x0"] + a["thickness"] + 2.0
        kw = a["half_width"] + 2.0
        part -= Pos((kx0 + kx1) / 2, 0, 0) * Box(kx1 - kx0, 2 * kw, 200.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return part


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
    comps = [Component("frame", frame(spec), material=g["frame_material"])]
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
    # (v2: the whole underside except round the bosses; the real parts reach x +-19.6, y +-18.6)
    under = Box(bx - 1.0, by - 1.0, g["board_underside_parts"], align=C)
    for sx in (-1, 1):
        for sy in (-1, 1):
            under -= Pos(sx * hx, sy * hy, 0) * Cylinder(g["board_boss_od"] / 2 + 0.5, g["board_underside_parts"], align=C)
    board += Pos(0, 0, -g["board_underside_parts"]) * under
    comps.append(Component("board", Pos(0, 0, d["board_z0"]) * board, mass_g=P["board"]["mass_g"],
                           group="avionics", touches={"frame"}))
    # TP4056 module on the board top (overhangs the right edge), under the ESP32
    cm = g["charger_module"]
    chg = Box(cm["x"][1] - cm["x"][0], cm["y"][1] - cm["y"][0], cm["height"], align=C)
    comps.append(Component("charger", Pos((cm["x"][0] + cm["x"][1]) / 2, (cm["y"][0] + cm["y"][1]) / 2, d["board_z1"]) * chg,
                           mass_g=P["charger"]["mass_g"], group="avionics", touches={"board"}))
    # ESP32 MiniKit on 8.5 mm female headers: the module occupies the top of the stack
    # (real outline and position from lite/ecad); the GY-521 sits under it on the board
    em = g["esp32_module"]
    hdr = g["esp32_header_height"]
    esp = Box(em["size"][0], em["size"][1], g["avionics_height"] - hdr, align=C)
    comps.append(Component("esp32", Pos(em["centre"][0], em["centre"][1], d["board_z1"] + hdr) * esp,
                           mass_g=P["esp32"]["mass_g"], group="avionics", touches={"board"}))
    im = g["imu_module"]
    imu = Box(im["x"][1] - im["x"][0], im["y"][1] - im["y"][0], im["height"], align=C)
    comps.append(Component("imu", Pos((im["x"][0] + im["x"][1]) / 2, (im["y"][0] + im["y"][1]) / 2, d["board_z1"]) * imu,
                           mass_g=P["imu"]["mass_g"], group="avionics", touches={"board", "charger"}))  # courtyards abut (board DRC owns that gap)
    au = g.get("autonomy", {})
    if au.get("fit"):
        for key, part, colour in (("flow_module", "flow", None), ("tof_module", "tof", None)):
            m = au[key]
            sx, sy, sz = m["size"]
            comps.append(Component(part, Pos(m["centre"][0], m["centre"][1], d["t"]) * Box(sx, sy, sz, align=C),
                                   mass_g=P[part]["mass_g"], group="autonomy", touches={"frame"}))
        cam = au["camera"]
        cy, cz, cdep = cam["size"]
        x1 = au["arch"]["x0"]
        comps.append(Component("camera", Pos(x1 - cdep / 2, cam["y_centre"], cam["z0"]) * Box(cdep, cy, cz, align=C),
                               mass_g=P["camera"]["mass_g"], group="autonomy", touches={"frame"}))
    if g["guard"]["fit"]:
        comps.append(Component("guard", guard(spec), material=g["frame_material"], touches={"frame"}))
    e = P["battery"]["envelope_mm"]
    comps.append(Component("battery", Pos(P["battery"].get("x_offset", 0.0), 0, d["t"] + 0.5) * Box(e["x"], e["y"], e["z"], align=C),
                           mass_g=P["battery"]["mass_g"], group="battery", touches={"frame"}))
    return comps


def lumped_masses(spec):
    d = dims(spec)
    zp = (d["prop_z0"] + d["prop_z1"]) / 2
    out = [(f"prop_{i}", spec["parts"]["prop"]["mass_g"], (x, y, zp)) for i, (x, y) in enumerate(motor_xy(spec))]
    out.append(("hardware", spec["parts"]["hardware"]["mass_g"], (0.0, 0.0, 2.0)))
    out.append(("wiring", spec["parts"]["wiring"]["mass_g"], (0.0, 0.0, 4.0)))
    if spec["geometry_mm"].get("autonomy", {}).get("fit"):
        out.append(("autonomy_wiring", spec["parts"]["autonomy_wiring"]["mass_g"], (-5.0, 0.0, 8.0)))  # leads run nose to tail
    return out

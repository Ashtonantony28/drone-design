"""Board geometry and placement of the Lite flight controller (shared by every layout script).

Plain Python (no pcbnew), so that both KiCad's Python (place.py, route_apply.py)
and host Python (router.py, plot.py, ../check_board.py) can import it.

Coordinates: millimetres from the board centre, x to the right, y DOWN, looking
at the TOP side (the KiCad convention). Drone axes: nose = +x, drone right = +y,
so the motors are M1 rear-right (-x,+y), M2 front-right (+x,+y), M3 rear-left
(-x,-y), M4 front-left (+x,-y) (Betaflight QUAD-X numbering).

Sides: through-hole parts sit on the top (F) and are soldered on the bottom; the
SMD parts sit on the bottom (B), on the copper. For B the rotation is the KiCad
orientation after flipping.
"""

ORIGIN = (100.0, 100.0)  # board centre on the KiCad sheet

BOARD_W, BOARD_H = 46.0, 38.0
CORNER_R = 3.0
HOLES = [(-19.0, -15.0), (19.0, -15.0), (19.0, 15.0), (-19.0, 15.0)]  # H1..H4, M2, frame lite/spec.yaml board_holes
HOLE_D = 2.2

# --- mill rules (MILLING.md; mirrored in the .kicad_pro by lite_fc.zen and in lite_fc.kicad_dru) ---
TRACK_MIN = 0.5
POWER_TRACK_MIN = 1.0
CLEARANCE = 0.4
DRILL_MIN = 0.8
EDGE_CLEARANCE = 0.5
HOLE_CLEARANCE = 0.5

# Nets that carry battery or motor current: netclass "Power", >= 1.0 mm
POWER_NETS = ["VBAT", "GND", "BAT_P", "BAT_N", "M1_N", "M2_N", "M3_N", "M4_N"]

# ESP32 MiniKit: inner rows at x = +-11.43, row r at y = U1_Y + (r - 5.5) * 2.54
U1_AT = (2.5, -4.15)
# TP4056 module: 26 x 17.5 mm, USB to +x; x from -8.8 (SW1 ends at -8.85) to 17.2 (H3 screw head
# starts at 17.25), y from 3.8 (the GY-521 ends at 3.5) to 21.3 (2.3 mm over the +y edge)
U3_AT = (4.2, 12.55)

# path (Zener instance path of the footprint) -> (x, y, rotation, side)
# Bottom-side two-pad parts: rotation 0 puts pad 1 at -x, 180 at +x, 90 at +y, 270 at -y.
# Bottom-side SOT-23 (1 G, 2 S, 3 D): rotation 180 puts the drain at -x, gate at (+x, -y), source at (+x, +y);
# rotation 0 the drain at +x, gate at (-x, +y), source at (-x, -y).
# Motor pads: rotation 0 puts M+ at (x, y) and M- at (x, y + 5); rotation 180 M- at (x, y - 5).
# Left motors have M+ at the corner, right motors M+ on the inside, so that VBAT runs straight
# down the right edge through M4+, C1+ and M2+.
PARTS = {
    "U1.X": (U1_AT[0], U1_AT[1], 0, "F"),  # inner rows at x = -8.93 / +13.93
    "U2.X": (-3.5, -15.54, 0, "F"),  # GY-521, pin 1 (VCC) here, body to +x
    "U3.X": (U3_AT[0], U3_AT[1], 0, "F"),
    "SW1.X": (-13.15, 17.6, 0, "F"),  # actuator over the bottom (drone-right) edge; throw B (VBAT) at +x
    "J1.X": (-13.0, 13.4, 180, "F"),  # battery
    "C1.X": (19.6, -0.5, 180, "F"),  # 470 uF bulk, + on the edge side
    "D5.X": (-16.6, -1.6, 270, "F"),  # status LED (K up), outside the ESP32 module so it can be seen
    "M1.X": (-21.2, 10.5, 180, "F"),  # rear-right: + at y 10.5, - at y 5.5
    "M2.X": (21.2, 5.5, 0, "F"),  # front-right: + at y 5.5, - at y 10.5
    "M3.X": (-21.2, -10.5, 0, "F"),  # rear-left: + at y -10.5, - at y -5.5
    "M4.X": (21.2, -5.5, 180, "F"),  # front-left: + at y -5.5, - at y -10.5
    # --- bottom side ---
    "JP1.X": (-9.6, 8.05, 270, "B"),  # BAT_N pad down towards J1 and B-
    # LDO under the TP4056 module, below the ESP32 3V3 pin
    "U4.X": (0.4, 5.2, 180, "B"),
    "C2.X": (4.0, 8.6, 0, "B"),
    "C3.X": (-3.0, 6.2, 270, "B"),
    "C4.X": (-1.1, -14.27, 270, "B"),  # at the IMU VCC / GND pins
    # battery divider at the top-left, next to IO36
    "R9.X": (-11.6, -17.5, 180, "B"),
    "R10.X": (-11.6, -14.9, 180, "B"),
    "R11.X": (-14.2, -1.9, 270, "B"),
    # motor switches: flyback, MOSFET, gate resistor, pull-down
    "D3.X": (-18.6, -8.0, 270, "B"), "Q3.X": (-15.45, -7.1, 180, "B"), "R3.X": (-12.9, -9.8, 180, "B"), "R7.X": (-14.1, -12.2, 0, "B"),
    "D1.X": (-18.6, 8.0, 90, "B"), "Q1.X": (-15.45, 7.6, 180, "B"), "R1.X": (-12.6, 3.8, 180, "B"), "R5.X": (-16.3, 3.4, 90, "B"),
    "D4.X": (18.6, -8.0, 90, "B"), "Q4.X": (11.5, -13.6, 0, "B"), "R4.X": (16.27, -7.6, 90, "B"), "R8.X": (6.4, -12.3, 180, "B"),
    "D2.X": (18.6, 8.0, 270, "B"), "Q2.X": (15.6, 10.8, 270, "B"), "R2.X": (14.6, 6.0, 270, "B"), "R6.X": (11.0, 7.8, 180, "B"),
}

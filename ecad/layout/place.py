#!/usr/bin/env python3
"""Place the nf_fc board: outline, mounting holes, parts, planes and keep-outs.

Runs under KiCad 10's Python (pcbnew). Re-runnable: it removes every track, via
and zone and re-places every footprint, so run layout/route.py afterwards.

    layout/run.sh place          (wraps: python3 layout/place.py <board.kicad_pcb>)

Coordinates in the tables are millimetres from the board centre, x to the right
and y down, looking at the TOP side. Parts are keyed by their Zener instance
path (the footprint "Path" field), which is stable; C/R designators are not.

Placement follows neuroflight-fc hw/DESIGN.md:
  section 1  36 x 36 mm, r = 3 mm, holes on a 30.5 mm square, MCU centre top
  section 4  IMU island top-left, diagonally opposite J2 + buck (bottom right, back)
  section 5  decoupling at the pins, solid L2 ground, buck + inductor on L4 by J2
"""
import json
import math
import os
import sys

import pcbnew

MM = pcbnew.FromMM
ORIGIN = (100.0, 100.0)  # board centre on the KiCad sheet

BOARD_SIZE = 36.0
CORNER_R = 3.0
HOLE_PITCH = 30.5
HOLE_KEEPOUT_R = 3.5  # DESIGN.md 1: about 7 mm keep-out circle, all layers
EDGE_INSET = 0.3

MCU = "MCU.U1.X"
MCU_AT = (2.0, 1.5)

# IMU island, top side (DESIGN.md 4). L3 below it is 3V3_IMU; ISLAND_FEED joins
# the LP5907 output (bottom side, outside the island) to it.
ISLAND = (-17.7, -13.0, -9.7, -3.3)  # x0, y0, x1, y1
ISLAND_FEED = (-9.7, -13.7, -5.9, -8.0)
ISLAND_RING = 1.0  # keep-out ring around the island
# VBAT has no L3 pour: every VBAT pad is on L4 in the J2/buck corner and is fed
# by the L4 pour that layout/route.py adds, which leaves L3 an unbroken 3V3_D
# reference under the L4 DShot traces.

# path -> (x, y, rotation in degrees, side). For "B" the rotation is the KiCad
# orientation after flipping.
PARTS = {
    MCU: (MCU_AT[0], MCU_AT[1], -90, "F"),
    # --- HSE (DESIGN.md 5.10: load caps next to the crystal) ---
    "MCU.Y1.Y": (1.65, -11.4, 90, "F"),
    "MCU.C_HSE_IN.C": (4.45, -10.25, 0, "F"),
    "MCU.C_HSE_OUT.C": (-1.15, -12.5, 0, "F"),
    "MCU.FB2.FB": (-2.6, -10.05, 0, "F"),
    "MCU.C_VDD_BULK.C": (-5.4, -8.3, 0, "F"),
    "MCU.C_NRST.C": (0.45, -8.3, 90, "F"),
    # --- receiver connector J3, top edge ---
    "CONN.J3.X": (7.5, -14.57, 180, "F"),
    "CONN.R_RC_TX.R": (7.05, -10.25, 90, "F"),
    "CONN.R_RC_RX.R": (5.95, -10.25, 90, "F"),
    # --- SWD Tag-Connect, top-left strip ---
    "CONN.J6.X": (-6.0, -15.0, 0, "F"),
    # --- baro, right strip top: away from the IMU island and the USB opening ---
    "BARO.U4.X": (15.0, -8.7, 180, "F"),
    "BARO.C_U4_VDD.C": (15.9, -10.6, 0, "F"),
    "BARO.C_U4_VDDIO.C": (13.9, -10.6, 0, "F"),
    "BARO.R_SCL.R": (11.5, -6.6, 90, "F"),
    "BARO.R_SDA.R": (12.6, -7.6, 90, "F"),
    # --- BOOT0 ---
    "MCU.SW1.X": (14.95, 0.5, 90, "F"),
    "MCU.R_BOOT0.R": (11.9, -1.5, 90, "F"),
    # --- FPGA straps next to their MCU pins ---
    "MCU.R_FPGA_IRQ.R": (11.9, 0.45, 90, "F"),
    "MCU.R_FPGA_CDONE.R": (11.9, 2.4, 90, "F"),
    "MCU.R_FPGA_CRESET.R": (16.2, -3.15, 0, "F"),
    "MCU.R_FPGA_SS.R": (14.2, -3.15, 0, "F"),
    # --- flow connector J4, bottom edge (back to back with USB-C) ---
    "CONN.J4.X": (14.57, 7.6, 90, "F"),
    "CONN.R_FLOW_TX.R": (9.9, 10.82, 0, "F"),
    "CONN.R_FLOW_RX.R": (9.9, 11.9, 0, "F"),
    # --- aux connector J5, left edge ---
    "CONN.J5.X": (-14.57, 6.4, -90, "F"),
    # --- LEDs and buzzer driver, bottom strip ---
    "UI.LED_PWR.LED": (2.0, 16.45, 0, "F"),
    "UI.LED_STATUS.LED": (5.4, 16.45, 0, "F"),
    "UI.LED_WARN.LED": (8.8, 16.45, 0, "F"),
    "UI.R_PWR.R": (2.0, 15.15, 0, "F"),
    "UI.R_STATUS.R": (5.4, 15.15, 0, "F"),
    "UI.R_WARN.R": (8.8, 15.15, 0, "F"),
    "UI.Q1.X": (3.0, 12.45, 0, "F"),
    "UI.R_GATE.R": (5.7, 12.3, 90, "F"),
    "UI.D2.X": (9.6, 13.5, 0, "F"),
    # --- IMU island: both IMUs with the same orientation ---
    "IMU.U2.X": (-11.45, -7.0, 180, "F"),
    "IMU.U3.X": (-15.3, -7.0, 180, "F"),
    "IMU.C_U2_VDD_100N.C": (-12.75, -9.65, 90, "F"),
    "IMU.C_U2_VDD_2U2.C": (-11.7, -9.65, 90, "F"),
    "IMU.C_U2_VDDIO.C": (-10.65, -9.65, 90, "F"),
    "IMU.C_U3_VDD.C": (-16.5, -9.65, 90, "F"),
    "IMU.C_U3_VDDIO.C": (-14.85, -9.65, 90, "F"),
    "IMU.R_IMUA_SCK.R": (-11.45, -4.35, 90, "F"),
    "IMU.R_IMUA_CS.R": (-12.55, -4.35, 90, "F"),
    "IMU.R_IMUB_SCK.R": (-15.3, -4.35, 90, "F"),
    "IMU.R_IMUB_CS.R": (-16.4, -4.35, 90, "F"),
    "IMU.TP_IMUA_CLKIN.TP": (-10.75, -12.0, 0, "F"),
    # --- FPGA (DNP) decoupling that does not fit beside U10 on the bottom ---
    "FPGA.C_VCCPLL.C": (12.2, -10.0, 90, "F"),
    "FPGA.C_VCCIO_0.C": (14.0, -5.3, 0, "F"),
    "FPGA.C_VCCIO_1.C": (16.0, -5.3, 0, "F"),
    "FPGA.C_VCCIO_2.C": (14.0, -4.2, 0, "F"),
    "FPGA.C_VPP.C": (16.0, -4.2, 0, "F"),
    # ================= bottom side =================
    "CONN.J1.X": (-6.0, 14.7, 180, "B"),
    "CONN.J2.X": (5.9, 14.55, 180, "B"),
    "CONN.J7.X": (-13.8, 5.0, 90, "B"),
    # --- USB ---
    "CONN.U9.X": (-6.0, 7.2, 90, "B"),
    "CONN.C_U9.C": (-8.8, 6.3, 90, "B"),
    "CONN.R_CC2.R": (-9.0, 8.4, 0, "B"),
    "CONN.R_CC1.R": (0.9, 10.0, 0, "B"),
    "PWR.D1.X": (-1.8, 7.9, 0, "B"),
    "SENSE.R_VBUS_TOP.R": (1.6, 7.0, 0, "B"),
    "SENSE.R_VBUS_BOT.R": (1.6, 8.1, 0, "B"),
    # --- aux series / pull-ups ---
    "CONN.R_AUX_TX.R": (-9.0, 3.2, 0, "B"),
    "CONN.R_AUX_RX.R": (-9.0, 4.3, 0, "B"),
    "CONN.R_AUX_SCL.R": (-6.9, 3.2, 0, "B"),
    "CONN.R_AUX_SDA.R": (-6.9, 4.3, 0, "B"),
    # --- DShot series resistors under the MCU pins (DESIGN.md 5.5) ---
    "CONN.R_M1.R": (-2.9, 1.9, 0, "B"),
    "CONN.R_M2.R": (-2.9, 2.95, 0, "B"),
    "CONN.R_M3.R": (-2.9, 4.0, 0, "B"),
    "CONN.R_M4.R": (-2.9, 5.05, 0, "B"),
    # --- 3V3_D LDO: pins up, tab down, away from the island ---
    "PWR.U7.X": (-4.3, -3.2, -90, "B"),
    "PWR.C_U7_IN.C": (0.0, -6.0, 90, "B"),
    "PWR.C_U7_OUT.C": (0.2, -1.5, 90, "B"),
    "SENSE.R_5V_TOP.R": (2.2, -1.6, 90, "B"),
    "SENSE.R_5V_BOT.R": (3.3, -1.6, 90, "B"),
    "SENSE.C_CUR.C": (-0.6, -8.3, 90, "F"),
    # --- IMU LDO, outside the island ---
    "PWR.U8.X": (-6.2, -10.0, 180, "B"),
    "PWR.FB1.FB": (-2.4, -9.3, 90, "B"),
    "PWR.C_U8_IN.C": (-3.1, -12.0, 0, "B"),
    "PWR.C_U8_OUT1.C": (-8.9, -10.9, 90, "B"),
    "PWR.C_U8_OUT2.C": (-7.6, -12.9, 0, "B"),
    # --- flash ---
    "FLASH.U5.X": (3.8, -10.4, 0, "B"),
    "FLASH.C_U5_VCC.C": (7.2, -6.5, 0, "B"),
    "FLASH.R_NCS.R": (0.0, -14.24, 0, "B"),
    "FLASH.R_IO2.R": (2.0, -14.24, 0, "B"),
    "FLASH.R_IO3.R": (5.5, -14.24, 0, "B"),
    # --- FPGA (DNP) ---
    "FPGA.U10.X": (12.7, -6.8, 0, "B"),
    "FPGA.C_VCC_5.C": (9.6, -12.45, 0, "B"),
    "FPGA.C_VCC_30.C": (11.6, -12.45, 0, "B"),
    "PWR.U11.X": (2.8, -4.9, 0, "B"),
    "PWR.C_U11_IN.C": (5.85, -5.4, 0, "B"),
    "PWR.C_U11_OUT.C": (5.85, -4.3, 0, "B"),
    # --- buck, by J2 (DESIGN.md 5.3) ---
    "PWR.U6.X": (8.6, 4.3, 180, "B"),
    "PWR.L1.X": (8.6, -0.3, 0, "B"),
    "PWR.C_VIN_HF1.C": (6.3, 3.55, 90, "B"),
    "PWR.C_VIN_HF2.C": (10.9, 3.55, 90, "B"),
    "PWR.C_VBAT1.C": (13.3, 3.6, 90, "B"),
    "PWR.C_VBAT2.C": (15.8, 3.6, 90, "B"),
    "PWR.C_VBAT3.C": (4.2, 3.6, 90, "B"),
    "PWR.C_5V1.C": (12.3, -0.55, 90, "B"),
    "PWR.C_5V2.C": (14.4, -0.55, 90, "B"),
    "PWR.C_5V3.C": (16.5, -0.55, 90, "B"),
    "PWR.C_BOOT.C": (11.1, 5.65, 0, "B"),
    "PWR.C_VCC.C": (9.2, 7.3, 90, "B"),
    "PWR.R_FBB.R": (8.1, 7.3, 90, "B"),
    "PWR.R_FBT.R": (7.0, 7.3, 90, "B"),
    "PWR.C_FF.C": (5.9, 7.3, 90, "B"),
    "PWR.D3.D": (13.6, 8.9, 0, "B"),
    "SENSE.R_VBAT_TOP.R": (9.35, 9.6, 90, "B"),
    "SENSE.R_VBAT_BOT.R": (8.25, 9.6, 90, "B"),
    "SENSE.R_CUR.R": (7.15, 9.6, 90, "B"),
    # --- test points (DESIGN.md 8: bottom side) and pads ---
    "TP1.TP": (5.57, 9.9, 0, "B"),  # VBAT
    "TP2.TP": (5.2, 0.2, 0, "B"),  # 5V
    "TP3.TP": (-0.3, 3.0, 0, "B"),  # 3V3_D
    "TP4.TP": (-10.9, -13.6, 0, "B"),  # 3V3_IMU
    "TP5.TP": (9.6, -14.4, 0, "B"),  # 1V2_FPGA
    "TP6.TP": (-1.2, -16.5, 0, "B"),  # VDDA
    "TP7.TP": (1.9, 1.6, 0, "B"),  # GND near TP2
    "TP15.TP": (-10.6, -15.9, 0, "B"),  # GND near TP4
    "PWR.TP8.TP": (5.15, -2.08, 0, "B"),  # buck SW
    "TP9.TP": (3.9, 7.1, 0, "B"),  # VBAT_SENSE
    "TP10.TP": (3.45, 9.7, 0, "B"),  # MOTOR0 after series R
    "TP11.TP": (-9.0, 0.3, 0, "B"),  # IMUA_INT1
    "TP12.TP": (0.1, 5.3, 0, "B"),  # IMUB_INT1
    "TP13.TP": (-6.0, -16.5, 0, "B"),  # NRST
    "TP14.TP": (9.4, -16.6, 0, "B"),  # BOOT0
    "UI.PAD_LED_STRIP.TP": (1.4, -16.1, 0, "B"),
    "UI.BZ_N.TP": (4.1, -16.1, 0, "B"),
    "UI.BZ_P.TP": (6.8, -16.1, 0, "B"),
}

# Two-pad parts keep the rotation above or turn 180 degrees, whichever brings
# their pads nearer to the pads they connect to (see auto_orient()).

# Decoupling at the MCU pins (DESIGN.md 5.1): capacitor path -> (MCU pad, offset
# along the pin row in mm). The capacitor sits perpendicular to the row, just
# outside the MCU courtyard, with its supply pad facing the pin.
MCU_DECAPS = {
    "MCU.C_VBAT_6.C": ("6", 0.0),
    "MCU.C_VDD_11.C": ("11", 0.0),
    "MCU.C_VDDA_100N.C": ("21", -0.25),  # straddles VREF+ (20) and VDDA (21)
    "MCU.C_VDDA_1U.C": ("21", -1.3),
    "MCU.C_VDD_27.C": ("27", 0.25),
    "MCU.C_VCAP_48.C": ("48", -0.1),
    "MCU.C_VDD_50.C": ("50", 0.1),
    "MCU.C_VCAP_73.C": ("73", -0.1),
    "MCU.C_VDD_75.C": ("75", 0.1),
    "MCU.C_VDD_100.C": ("100", 0.0),
}
DECAP_GAP = 1.32  # pad end to capacitor centre: clears the two courtyards

# Nets routed as wide tracks (Zener netclass "Power")
POWER_NETS = ["5V", "VBUS", "VBAT", "PWR.IMU_LDO_IN"]

# Reference designators shown on silkscreen; everything else is on the fab layer
SILK_REFS = ("J", "TP", "SW", "U1")


def pt(x, y):
    return pcbnew.VECTOR2I(MM(ORIGIN[0] + x), MM(ORIGIN[1] + y))


def rel(v):
    return (pcbnew.ToMM(v.x) - ORIGIN[0], pcbnew.ToMM(v.y) - ORIGIN[1])


def footprints_by_path(footprints):
    out = {}
    for fp in footprints:
        path = next((f.GetText() for f in fp.GetFields() if f.GetName() == "Path"), fp.GetReference())
        out[path] = fp
    return out


def put(fp, x, y, rot, side):
    if fp.IsFlipped() != (side == "B"):
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    fp.SetPosition(pt(x, y))
    fp.SetOrientationDegrees(rot)


def pad_xy(fp, number):
    pads = [p for p in fp.Pads() if p.GetNumber() == number]
    assert pads, "%s has no pad %s" % (fp.GetReference(), number)
    return rel(pads[0].GetPosition()), pads[0]


def place_decap(cap, mcu, pad_number, along):
    (px, py), pad = pad_xy(mcu, pad_number)
    cx, cy = rel(mcu.GetPosition())
    half_len = max(pcbnew.ToMM(pad.GetSize().x), pcbnew.ToMM(pad.GetSize().y)) / 2
    if abs(px - cx) > abs(py - cy):  # left or right row
        sx = 1 if px > cx else -1
        x, y, rots = px + sx * (half_len + DECAP_GAP), py + along, (0, 180)
    else:
        sy = 1 if py > cy else -1
        x, y, rots = px + along, py + sy * (half_len + DECAP_GAP), (90, 270)
    # choose the rotation that puts the pad on the pin's net next to the pin
    best = None
    for rot in rots:
        put(cap, x, y, rot, "F")
        for p in cap.Pads():
            if p.GetNetname() == pad.GetNetname():
                qx, qy = rel(p.GetPosition())
                d = math.hypot(qx - px, qy - py)
                if best is None or d < best[0]:
                    best = (d, rot)
    assert best, "%s shares no net with MCU pad %s" % (cap.GetReference(), pad_number)
    put(cap, x, y, best[1], "F")


PLANE_NETS = ("GND", "3V3_D", "3V3_IMU")


def auto_orient(footprints, skip):
    """Turn two-pad parts by 180 degrees when that shortens their connections.

    Cost = distance from each pad to the nearest pad of the same net on another
    footprint; pads on plane nets cost nothing (they drop a via in place).
    """
    by_net = {}
    for fp in footprints:
        for p in fp.Pads():
            if p.GetNetname() and p.GetNetname() not in PLANE_NETS:
                by_net.setdefault(p.GetNetname(), []).append((fp.GetReference(), p))

    def cost(fp):
        total = 0.0
        for p in fp.Pads():
            others = [q for ref, q in by_net.get(p.GetNetname(), []) if ref != fp.GetReference()]
            if others:
                a = p.GetPosition()
                total += min(math.hypot(a.x - q.GetPosition().x, a.y - q.GetPosition().y) for q in others)
        return total

    skip_refs = {fp.GetReference() for fp in skip}
    for fp in footprints:
        if fp.GetPadCount() != 2 or fp.GetReference() in skip_refs:
            continue
        rot = fp.GetOrientationDegrees()
        before = cost(fp)
        fp.SetOrientationDegrees(rot + 180)
        if cost(fp) >= before - 1e3:  # nm; keep the table's rotation on a tie
            fp.SetOrientationDegrees(rot)


def set_outline(board):
    h, r = BOARD_SIZE / 2, CORNER_R

    def line(a, b):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pt(*a))
        s.SetEnd(pt(*b))
        return s

    def arc(start, mid, end):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_ARC)
        s.SetArcGeometry(pt(*start), pt(*mid), pt(*end))
        return s

    k = r * (1 - math.sqrt(0.5))
    shapes = [
        line((-h + r, -h), (h - r, -h)), arc((h - r, -h), (h - k, -h + k), (h, -h + r)),
        line((h, -h + r), (h, h - r)), arc((h, h - r), (h - k, h - k), (h - r, h)),
        line((h - r, h), (-h + r, h)), arc((-h + r, h), (-h + k, h - k), (-h, h - r)),
        line((-h, h - r), (-h, -h + r)), arc((-h, -h + r), (-h + k, -h + k), (-h + r, -h)),
    ]
    for s in shapes:
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(MM(0.1))
        board.Add(s)


def add_silk(board):
    # DESIGN.md 1: arrow marks body +x (forward) = towards the top edge
    def seg(a, b):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pt(*a))
        s.SetEnd(pt(*b))
        s.SetLayer(pcbnew.F_SilkS)
        s.SetWidth(MM(0.15))
        board.Add(s)

    x, y0, y1 = -1.6, -14.2, -16.6
    seg((x, y0), (x, y1))
    seg((x, y1), (x - 0.6, y1 + 0.8))
    seg((x, y1), (x + 0.6, y1 + 0.8))
    for text, x, y, layer in (("+X", -1.6, -17.2, pcbnew.F_SilkS), ("NF-FC v1", 11.5, 12.3, pcbnew.B_SilkS)):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(text)
        t.SetLayer(layer)
        t.SetPosition(pt(x, y))
        t.SetTextSize(pcbnew.VECTOR2I(MM(0.7), MM(0.7)))
        t.SetTextThickness(MM(0.12))
        if layer == pcbnew.B_SilkS:
            t.SetMirrored(True)
        board.Add(t)


def tidy_text(footprints):
    """Small parts get their designator on the fab layer; the rest a 0.6 mm silk label."""
    for fp in footprints:
        ref = fp.Reference()
        keep = fp.GetReference().startswith(SILK_REFS)
        fab = pcbnew.B_Fab if fp.IsFlipped() else pcbnew.F_Fab
        silk = pcbnew.B_SilkS if fp.IsFlipped() else pcbnew.F_SilkS
        ref.SetLayer(silk if keep else fab)
        ref.SetTextSize(pcbnew.VECTOR2I(MM(0.6), MM(0.6)))
        ref.SetTextThickness(MM(0.1))
        ref.SetVisible(True)
        fp.Value().SetLayer(fab)
        fp.Value().SetVisible(False)
        if fp.GetReference().startswith("TP"):
            ref.SetPosition(fp.GetPosition() + pcbnew.VECTOR2I(0, MM(-1.15)))
        elif not keep:
            ref.SetPosition(fp.GetPosition())


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def add_zone(board, net, layer, outline, priority=0, name="", solid=True, clearance=0.15):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNetCode(board.GetNetcodeFromNetname(net))
    z.SetZoneName(name or net)
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(MM(clearance))
    z.SetMinThickness(MM(0.15))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if solid else pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(MM(0.15))
    z.SetThermalReliefSpokeWidth(MM(0.25))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    poly = z.Outline()
    poly.NewOutline()
    for x, y in outline:
        poly.Append(MM(ORIGIN[0] + x), MM(ORIGIN[1] + y))
    board.Add(z)
    return z


def add_keepout(board, name, outline, layers, tracks=True, vias=True, pours=True, footprints=False):
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetZoneName(name)
    ls = pcbnew.LSET()
    for layer in layers:
        ls.AddLayer(layer)
    z.SetLayerSet(ls)
    z.SetDoNotAllowTracks(tracks)
    z.SetDoNotAllowVias(vias)
    z.SetDoNotAllowZoneFills(pours)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(footprints)
    poly = z.Outline()
    poly.NewOutline()
    for x, y in outline:
        poly.Append(MM(ORIGIN[0] + x), MM(ORIGIN[1] + y))
    board.Add(z)
    return z


def l3_3v3_outline():
    """3V3_D on L3: the board minus the 3V3_IMU island, its feed and the 1 mm
    ring around them (DESIGN.md 4).

    Written as one explicit polygon so that no two L3 zones overlap: the
    autorouter is told about each plane by its outline.
    """
    e = BOARD_SIZE / 2 - EDGE_INSET
    g = ISLAND_RING
    ix0, iy0, ix1, iy1 = ISLAND
    fx0, fy0, fx1, fy1 = ISLAND_FEED
    return [
        (-e, -e), (e, -e), (e, e), (-e, e),
        # left edge, stepping around the island and its feed
        (-e, iy1 + g), (ix1 + g, iy1 + g), (ix1 + g, fy1 + g), (fx1 + g, fy1 + g),
        (fx1 + g, fy0 - g), (ix1 - g, fy0 - g), (ix1 - g, iy0 - g), (-e, iy0 - g),
    ]


def island_outline():
    ix0, iy0, ix1, iy1 = ISLAND
    fx0, fy0, fx1, fy1 = ISLAND_FEED
    return [(ix0, iy0), (ix1, iy0), (ix1, fy0), (fx1, fy0), (fx1, fy1), (ix1, fy1), (ix1, iy1), (ix0, iy1)]


def add_planes(board):
    e = BOARD_SIZE / 2 - EDGE_INSET
    # L2: one solid, unbroken ground plane (DESIGN.md 5.4)
    add_zone(board, "GND", pcbnew.In1_Cu, rect(-e, -e, e, e), name="L2_GND")
    # L3: power pours
    add_zone(board, "3V3_D", pcbnew.In2_Cu, l3_3v3_outline(), name="L3_3V3_D")
    add_zone(board, "3V3_IMU", pcbnew.In2_Cu, island_outline(), name="L3_3V3_IMU")


def add_keepouts(board):
    cu = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
    q = HOLE_PITCH / 2
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
        circle = [(sx * q + HOLE_KEEPOUT_R * math.cos(a * math.pi / 18), sy * q + HOLE_KEEPOUT_R * math.sin(a * math.pi / 18)) for a in range(36)]
        add_keepout(board, "HOLE%d_KEEPOUT" % (i + 1), circle, cu)
    # DESIGN.md 4: nothing but the island's own nets under the IMUs. Signal
    # layers below the island: no tracks or vias on L4, and no L4 pour.
    add_keepout(board, "IMU_ISLAND_L4", rect(*ISLAND), [pcbnew.B_Cu], vias=False)


def assign_netclasses(pro_path):
    with open(pro_path) as f:
        pro = json.load(f)
    patterns = pro["net_settings"].setdefault("netclass_patterns", [])
    patterns[:] = [p for p in patterns if p.get("netclass") != "Power"]
    patterns.extend({"netclass": "Power", "pattern": n} for n in POWER_NETS)
    with open(pro_path, "w") as f:
        json.dump(pro, f, indent=2)
        f.write("\n")


def main(board_path):
    assign_netclasses(os.path.splitext(board_path)[0] + ".kicad_pro")
    board = pcbnew.LoadBoard(board_path)
    # Take every container before the first Remove(): with KiCad 10.0.8 the
    # SWIG list accessors stop working once an item has been removed.
    footprints = list(board.GetFootprints())
    stale = list(board.GetTracks()) + list(board.Zones())
    stale += [d for d in board.GetDrawings() if d.GetLayer() in (pcbnew.Edge_Cuts, pcbnew.F_SilkS, pcbnew.B_SilkS)]
    for item in stale:
        board.Remove(item)
    set_outline(board)
    fps = footprints_by_path(footprints)

    q = HOLE_PITCH / 2
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
        put(fps.pop("H%d" % (i + 1)), sx * q, sy * q, 0, "F")
    mcu = fps[MCU]
    put(mcu, *PARTS[MCU])
    decaps = [fps.pop(path) for path in MCU_DECAPS]
    for fp, (pad, along) in zip(decaps, MCU_DECAPS.values()):
        place_decap(fp, mcu, pad, along)
    for path, (x, y, rot, side) in PARTS.items():
        put(fps.pop(path), x, y, rot, side)
    assert not fps, "unplaced footprints: %s" % sorted(fps)
    auto_orient(footprints, skip=decaps)

    for fp in footprints:
        fp.SetLocked(True)
    tidy_text(footprints)
    add_silk(board)
    add_planes(board)
    add_keepouts(board)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(board_path)
    print("placed %d footprints" % len(footprints))


if __name__ == "__main__":
    main(sys.argv[1])

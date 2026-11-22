#!/usr/bin/env python3
"""Generate the symbols and footprints of the Lite flight controller (output is committed).

    python3 lib/gen_lib.py            (from lite/ecad/)

Every footprint here is drawn for a desktop CNC isolation mill, not a fab house
(lite/ecad/MILLING.md): round/oval through-hole pads of 1.8 mm or more on a
1.0 mm drill (0.8 mm minimum), SMD pads enlarged so that every pad-to-pad gap is
at least 0.4 mm, and only the forgiving SMD packages the brief allows (SOT-23,
SOT-23-5, 1206, SMA/SOD-123). Through-hole pads are on both copper layers so that
a student using double-sided stock can mill an isolation ring on the top too;
all tracks are on B.Cu (the DRC rules forbid F.Cu tracks).

Symbols are plain boxes: pin i of a symbol is pad PADS[i] of its footprint.

Mechanical data that could not be checked against a drawing is marked VERIFY
(collected in lite/ecad/MILLING.md, "Things to check before you cut").
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
P = 2.54

# --------------------------------------------------------------------------------------
# footprint writer


def _f(v):
    s = ("%.4f" % v).rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


class Fp:
    def __init__(self, name, descr, tht):
        self.name, self.descr, self.is_tht = name, descr, tht
        self.items = []

    def line(self, x1, y1, x2, y2, layer="F.SilkS", w=0.12):
        self.items.append('\t(fp_line (start %s %s) (end %s %s) (stroke (width %s) (type solid)) (layer "%s"))'
                          % (_f(x1), _f(y1), _f(x2), _f(y2), _f(w), layer))

    def rect(self, x1, y1, x2, y2, layer="F.SilkS", w=0.12):
        for a, b, c, d in ((x1, y1, x2, y1), (x2, y1, x2, y2), (x2, y2, x1, y2), (x1, y2, x1, y1)):
            self.line(a, b, c, d, layer, w)

    def circle(self, x, y, r, layer="F.SilkS", w=0.12):
        self.items.append('\t(fp_circle (center %s %s) (end %s %s) (stroke (width %s) (type solid)) (fill no) (layer "%s"))'
                          % (_f(x), _f(y), _f(x + r), _f(y), _f(w), layer))

    def text(self, s, x, y, layer="F.SilkS", size=0.8):
        self.items.append('\t(fp_text user "%s" (at %s %s) (layer "%s") (effects (font (size %s %s) (thickness %s))))'
                          % (s, _f(x), _f(y), layer, _f(size), _f(size), _f(size * 0.15)))

    def tht(self, num, x, y, sx, sy=None, drill=1.0, shape=None):
        sy = sx if sy is None else sy
        shape = shape or ("circle" if sx == sy else "oval")
        self.items.append('\t(pad "%s" thru_hole %s (at %s %s) (size %s %s) (drill %s) (layers "*.Cu" "*.Mask"))'
                          % (num, shape, _f(x), _f(y), _f(sx), _f(sy), _f(drill)))

    def smd(self, num, x, y, sx, sy):
        self.items.append('\t(pad "%s" smd roundrect (at %s %s) (size %s %s) (layers "F.Cu" "F.Mask" "F.Paste") (roundrect_rratio 0.15))'
                          % (num, _f(x), _f(y), _f(sx), _f(sy)))

    def npth(self, x, y, d):
        self.items.append('\t(pad "" np_thru_hole circle (at %s %s) (size %s %s) (drill %s) (layers "*.Cu" "*.Mask"))'
                          % (_f(x), _f(y), _f(d), _f(d), _f(d)))

    def write(self, ref_at=(0, -3), val_at=(0, 3)):
        head = [
            '(footprint "%s"' % self.name,
            "\t(version 20240108)",
            '\t(generator "lite_gen_lib")',
            '\t(layer "F.Cu")',
            '\t(descr "%s")' % self.descr,
            '\t(property "Reference" "REF**" (at %s %s 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))'
            % (_f(ref_at[0]), _f(ref_at[1])),
            '\t(property "Value" "%s" (at %s %s 0) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))'
            % (self.name, _f(val_at[0]), _f(val_at[1])),
            "\t(attr %s)" % ("through_hole" if self.is_tht else "smd"),
        ]
        path = os.path.join(HERE, "footprints", self.name + ".kicad_mod")
        with open(path, "w") as f:
            f.write("\n".join(head + self.items) + "\n)\n")


# --------------------------------------------------------------------------------------
# footprints

HDR_PAD, HDR_DRILL = 1.8, 1.0  # 2.54 mm headers: 0.74 mm between pads


def esp32_minikit():
    """MH-ET LIVE ESP32 MiniKit ("ESP32 D1 mini") on its two INNER pin rows only.

    Origin: centre of the module's 4 x 10 pin grid. Inner rows at x = +-11.43 mm
    (0.9 in, the Wemos D1 mini spacing), rows 2.54 mm apart, row 1 at the antenna
    end. Pinout: RIOT-OS board doc + pinout drawing (esp32-mh-et-live-minikit),
    inner-left column RST SVP IO26 IO18 IO19 IO23 IO5 3V3 TCK SD3, inner-right
    TXD RXD IO22 IO21 IO17 IO16 GND VCC TD0 SD0. Only the rows this board uses get
    holes: left rows 2-8 (cut a 1x7 female header), right rows 3-8 (1x6). RST,
    TXD, RXD and rows 9-10 hang free. The outer rows are not used; solder no male
    pins there (or cut them flush).
    VERIFY: row/column positions against your module (calipers: 22.86 mm between
    the inner rows, row 1 about 7.7 mm from the antenna end).
    """
    fp = Fp("ESP32_MiniKit_InnerRows_Mill", "MH-ET LIVE ESP32 MiniKit, inner rows (L2-L8, R3-R8), 2.54 mm, mill pads", True)
    row = lambda r: (r - 5.5) * P
    for r in range(2, 9):
        fp.tht("L%d" % r, -11.43, row(r), HDR_PAD, drill=HDR_DRILL)
    for r in range(3, 9):
        fp.tht("R%d" % r, 11.43, row(r), HDR_PAD, drill=HDR_DRILL)
    # female header bodies (silk) and courtyard: the module itself is 8.5 mm up
    fp.rect(-12.7, row(2) - 1.27, -10.16, row(8) + 1.27)
    fp.rect(10.16, row(3) - 1.27, 12.7, row(8) + 1.27)
    fp.rect(-12.95, row(2) - 1.52, -9.91, row(8) + 1.52, "F.CrtYd", 0.05)
    fp.rect(9.91, row(3) - 1.52, 12.95, row(8) + 1.52, "F.CrtYd", 0.05)
    # module outline 31 x 39 mm on Fab, antenna end up
    top, bot = row(1) - 7.7, row(10) + 8.6
    fp.rect(-15.5, top, 15.5, bot, "F.Fab", 0.1)
    fp.rect(-9, top, 9, top + 6, "F.Fab", 0.1)
    fp.text("ANTENNA", 0, top + 3, "F.Fab", 1.0)
    fp.rect(-4, bot - 5, 4, bot + 1, "F.Fab", 0.1)
    fp.text("USB", 0, bot - 2, "F.Fab", 1.0)
    for r, s in ((2, "36"), (3, "26"), (4, "18"), (5, "19"), (6, "23"), (7, "5"), (8, "3V3")):
        fp.text(s, -14.6, row(r), size=0.7)
    for r, s in ((3, "22"), (4, "21"), (5, "17"), (6, "16"), (7, "G"), (8, "5V")):
        fp.text(s, 14.4, row(r), size=0.7)
    fp.text("ESP32 MiniKit", 0, row(1) - 1.0, size=1.0)
    fp.write(ref_at=(0, row(1) - 2.6), val_at=(0, bot - 7))


def gy521():
    """GY-521 (MPU-6050) breakout on its 1x8 header, pin 1 (VCC) at the origin.

    Pins run +y in the order VCC GND SCL SDA XDA XCL AD0 INT. Mounted component side
    up, header along the left edge: the board body then lies to the RIGHT (+x) of
    the pin row with VCC at the top (the breakout's usual picture, pins at the
    bottom and VCC on the left, turned 90 degrees clockwise).
    VERIFY: GY-521 outline 20.3 x 15.6 mm, pins centred on the long edge, 1.3 mm in.
    """
    fp = Fp("GY521_MPU6050_Mill", "GY-521 MPU-6050 breakout, 1x8 2.54 mm header, body to +x", True)
    names = ["VCC", "GND", "SCL", "SDA", "XDA", "XCL", "AD0", "INT"]
    for i, n in enumerate(names):
        fp.tht(str(i + 1), 0, i * P, HDR_PAD, drill=HDR_DRILL, shape="rect" if i == 0 else None)
        fp.text(n, 2.6, i * P, size=0.7)
    y0, y1 = -(20.3 - 7 * P) / 2, 7 * P + (20.3 - 7 * P) / 2
    fp.rect(-1.3, y0, 14.3, y1)
    fp.rect(-1.55, y0 - 0.25, 14.55, y1 + 0.25, "F.CrtYd", 0.05)
    fp.rect(-1.3, y0, 14.3, y1, "F.Fab", 0.1)
    fp.text("MPU-6050", 8.5, 7 * P / 2, "F.SilkS", 1.0)
    fp.write(ref_at=(8.5, 7 * P / 2 - 2), val_at=(8.5, 7 * P / 2 + 2))


def tp4056():
    """TP4056 charger with DW01 protection ("03962A"), mounted component side up.

    Origin: module centre, micro-USB end towards +x. Only the three pads this board
    needs get holes: B+ (battery +, internally the same copper as OUT+), B- (battery
    -) and OUT- (protected ground = board GND). OUT+, IN+ and IN- are not used; hold
    the USB end down with foam tape. Solder short wire legs (resistor offcuts)
    from the module pads into these holes, so a millimetre of pad-position error
    does not matter.
    VERIFY: outline 26 x 17.5 mm and the pad order OUT- / B- / B+ / (OUT+) along
    the battery end (looking at the component side with USB on the right, from
    the top: OUT-, B-, B+, OUT+). Check your module's silkscreen before soldering.
    """
    fp = Fp("TP4056_Protected_Module_Mill", "TP4056 + DW01 charger module (03962A), 3 used pads, mill", True)
    x = -13.0 + 1.3
    for num, y, lab in (("1", 2.0, "B+"), ("2", -2.0, "B-"), ("3", -6.4, "OUT-")):
        fp.tht(num, x, y, 2.2, drill=1.0)
        fp.text(lab, x + 2.8, y, size=0.8)
    fp.rect(-13, -8.75, 13, 8.75)
    fp.rect(-13.0, -8.75, 13.0, 8.75, "F.CrtYd", 0.05)  # body (tight: between SW1 and the H3 screw head)
    fp.rect(-13, -8.75, 13, 8.75, "F.Fab", 0.1)
    fp.rect(9.5, -4, 14, 4, "F.Fab", 0.1)
    fp.text("USB", 11.5, 0, "F.SilkS", 1.0)
    fp.text("TP4056", 0, 0, "F.SilkS", 1.2)
    fp.write(ref_at=(0, -3), val_at=(0, 3))


def jst_ph2():
    """JST PH 2.0 mm, 2 pin, vertical (B2B-PH-K). Pin 1 at -x.
    VERIFY battery lead polarity: many 1S packs swap red/black on PH plugs."""
    fp = Fp("JST_PH_B2B_Vertical_Mill", "JST PH 2.0 mm 2-pin vertical, oval mill pads", True)
    fp.tht("1", -1.0, 0, 1.4, 2.6, drill=0.9)
    fp.tht("2", 1.0, 0, 1.4, 2.6, drill=0.9)
    fp.rect(-3.0, -1.7, 3.0, 2.8)
    fp.rect(-3.25, -1.95, 3.25, 3.05, "F.CrtYd", 0.05)
    fp.text("-", -1.0, -2.6, size=1.0)
    fp.text("+", 1.0, -2.6, size=1.0)
    fp.write(ref_at=(0, 4.2), val_at=(0, -4))


def switch_spdt():
    """SS-12D00 style SPDT slide switch, 3 pins, actuator towards +y (to the board edge).
    VERIFY: lead pitch (2.54 mm assumed; some SS12D00 variants are 2.0 mm, bend the
    outer leads) and the current rating (the motors draw several amps through it)."""
    fp = Fp("SW_SPDT_SS12D00_Mill", "SPDT slide switch SS-12D00, 3 pins 2.54 mm, mill pads", True)
    for i, x in enumerate((-P, 0, P)):
        fp.tht(str(i + 1), x, 0, HDR_PAD, drill=1.0)
    fp.rect(-4.3, -1.75, 4.3, 1.75)
    fp.rect(-1.5, 1.75, 1.5, 3.75)  # actuator
    fp.rect(-4.3, -1.75, 4.3, 3.75, "F.CrtYd", 0.05)  # body and actuator (tight: SW1 sits between H4 and U3)
    fp.write(ref_at=(0, -3), val_at=(0, 5))


def cp_radial():
    """Electrolytic, 6.3 mm diameter, 2.5 mm lead spacing; pad 1 = +."""
    fp = Fp("CP_Radial_D6.3_P2.5_Mill", "Electrolytic radial D6.3 P2.5, mill pads", True)
    fp.tht("1", -1.25, 0, 1.9, drill=0.8, shape="rect")
    fp.tht("2", 1.25, 0, 1.9, drill=0.8)
    fp.circle(0, 0, 3.15)
    fp.circle(0, 0, 3.25, "F.CrtYd", 0.05)
    fp.text("+", -2.3, -1.9, size=1.0)
    fp.write(ref_at=(0, -4.2), val_at=(0, 4.2))


def led3():
    """3 mm through-hole LED; pad 1 = cathode (flat side, short leg)."""
    fp = Fp("LED_D3.0_Mill", "3 mm LED, 2.54 mm, mill pads", True)
    fp.tht("1", -1.27, 0, 1.8, drill=0.8, shape="rect")
    fp.tht("2", 1.27, 0, 1.8, drill=0.8)
    fp.circle(0, 0, 1.9)
    fp.line(-1.9, -1.2, -1.9, 1.2)
    fp.circle(0, 0, 2.0, "F.CrtYd", 0.05)
    fp.write(ref_at=(0, -3), val_at=(0, 3))


def motor_pads():
    """Two solder pads for a brushed motor's leads: 1 = M+ (VBAT, red), 2 = M- (to the MOSFET drain, black/blue).
    5 mm apart so a 1 mm power track can run between them."""
    fp = Fp("MotorPads_2_P5.0_Mill", "Brushed motor wire pads, 2 x 2.5 mm, 1.0 mm drill, 5.0 mm", True)
    fp.tht("1", 0, 0, 2.5, drill=1.0, shape="rect")
    fp.tht("2", 0, 5.0, 2.5, drill=1.0)
    fp.text("+", -2.2, 0, size=1.0)
    fp.text("-", -2.2, 5.0, size=1.0)
    fp.rect(-1.5, -1.5, 1.5, 6.5, "F.CrtYd", 0.05)
    fp.write(ref_at=(2.8, 2.5), val_at=(0, 8))


def sot23():
    """SOT-23 (AO3400A: 1 G, 2 S, 3 D), pads enlarged for milling (0.9 mm gaps)."""
    fp = Fp("SOT-23_Mill", "SOT-23, enlarged pads for an isolation mill", False)
    fp.smd("1", -1.25, -0.95, 1.5, 1.0)
    fp.smd("2", -1.25, 0.95, 1.5, 1.0)
    fp.smd("3", 1.25, 0, 1.5, 1.4)
    fp.rect(-0.7, -1.5, 0.7, 1.5, "F.Fab", 0.1)
    fp.rect(-2.1, -1.6, 2.1, 1.6, "F.CrtYd", 0.05)
    fp.line(-0.1, -1.6, 0.6, -1.6)
    fp.line(-0.1, 1.6, 0.6, 1.6)
    fp.write(ref_at=(0, -2.6), val_at=(0, 2.6))


def sot23_5():
    """SOT-23-5 (ME6211: 1 VIN, 2 VSS, 3 CE, 4 NC, 5 VOUT). Pads trimmed on the inner
    side (0.75 mm from the centre line) so that a 0.5 mm track fits under the body:
    pin 3 (CE) is tied to pin 1 (VIN) that way. 0.41 mm between pads in a row."""
    fp = Fp("SOT-23-5_Mill", "SOT-23-5, pads for an isolation mill, room under the body", False)
    x, sx, sy = 1.325, 1.15, 0.54
    for num, px, py in (("1", -x, -0.95), ("2", -x, 0), ("3", -x, 0.95), ("4", x, 0.95), ("5", x, -0.95)):
        fp.smd(num, px, py, sx, sy)
    fp.rect(-0.8, -1.45, 0.8, 1.45, "F.Fab", 0.1)
    fp.rect(-2.0, -1.55, 2.0, 1.55, "F.CrtYd", 0.05)
    fp.circle(-2.2, -1.6, 0.15)
    fp.write(ref_at=(0, -2.6), val_at=(0, 2.6))


def r1206(name="R_1206_Mill"):
    fp = Fp(name, "1206 resistor/capacitor, mill pads (1.6 mm gap)", False)
    fp.smd("1", -1.6, 0, 1.6, 1.9)
    fp.smd("2", 1.6, 0, 1.6, 1.9)
    fp.rect(-1.6, -0.8, 1.6, 0.8, "F.Fab", 0.1)
    fp.rect(-2.5, -1.05, 2.5, 1.05, "F.CrtYd", 0.05)
    fp.write(ref_at=(0, -2.1), val_at=(0, 2.1))


def diode_combo():
    """Schottky B5819W (SOD-123, 1.1 mm tall); the long pads also take an SMA (SS14).

    Pad 1 = cathode. The flyback diodes sit under the battery, which leaves about
    1.5 mm below the board (lite/spec.yaml): an SMA body (2.3 mm) does not fit there,
    so the courtyard is the SOD-123 one.
    """
    fp = Fp("D_SMA_SOD-123_Mill", "Diode, SOD-123 (or SMA) on the same pads, mill", False)
    fp.smd("1", -2.3, 0, 2.0, 1.8)
    fp.smd("2", 2.3, 0, 2.0, 1.8)
    fp.rect(-1.35, -0.8, 1.35, 0.8, "F.Fab", 0.1)
    fp.line(-0.7, -0.95, -0.7, 0.95)
    fp.rect(-3.4, -1.0, 3.4, 1.0, "F.CrtYd", 0.05)
    fp.write(ref_at=(0, -2.0), val_at=(0, 2.0))


def solder_jumper():
    """Two-pad solder jumper, open by default, 0.5 mm gap: bridge with a blob or a wire offcut."""
    fp = Fp("SolderJumper_2_Open_Mill", "Solder jumper, 2 pads, 0.5 mm gap, mill", False)
    fp.smd("1", -1.0, 0, 1.5, 2.0)
    fp.smd("2", 1.0, 0, 1.5, 2.0)
    fp.rect(-1.85, -1.1, 1.85, 1.1, "F.CrtYd", 0.05)
    fp.write(ref_at=(0, -2.2), val_at=(0, 2.2))


def mounting_hole():
    """M2 clearance hole, 2.2 mm NPTH, no copper.

    Top courtyard: the screw head, 3.5 mm (ISO 7380 button head M2; a 3.8 mm cheese
    head touches SW1 and the TP4056 module). Bottom courtyard: the frame boss, 5 mm OD.
    """
    fp = Fp("MountingHole_2.2mm_M2_Mill", "M2 mounting hole, 2.2 mm NPTH, 3.5 mm head / 5 mm boss keep-out", True)
    fp.npth(0, 0, 2.2)
    fp.circle(0, 0, 1.75)
    fp.circle(0, 0, 1.75, "F.CrtYd", 0.05)
    fp.circle(0, 0, 2.6, "B.CrtYd", 0.05)
    fp.write(ref_at=(0, -3.4), val_at=(0, 3.4))


# --------------------------------------------------------------------------------------
# symbols: one box per part, all pins on the left

SYMBOLS = {
    # name: (reference prefix, [(pin number, pin name)])
    "ESP32_MiniKit": ("U", [("L2", "IO36"), ("L3", "IO26"), ("L4", "IO18"), ("L5", "IO19"), ("L6", "IO23"),
                            ("L7", "IO5"), ("L8", "3V3"), ("R3", "IO22"), ("R4", "IO21"), ("R5", "IO17"),
                            ("R6", "IO16"), ("R7", "GND"), ("R8", "5V")]),
    "GY521": ("U", [("1", "VCC"), ("2", "GND"), ("3", "SCL"), ("4", "SDA"), ("5", "XDA"), ("6", "XCL"),
                    ("7", "AD0"), ("8", "INT")]),
    "TP4056_Module": ("U", [("1", "B+"), ("2", "B-"), ("3", "OUT-")]),
    "ME6211": ("U", [("1", "VIN"), ("2", "VSS"), ("3", "CE"), ("4", "NC"), ("5", "VOUT")]),
    "NMOS_GSD": ("Q", [("1", "G"), ("2", "S"), ("3", "D")]),
    "R": ("R", [("1", "1"), ("2", "2")]),
    "C": ("C", [("1", "1"), ("2", "2")]),
    "CP": ("C", [("1", "+"), ("2", "-")]),
    "D_Schottky": ("D", [("1", "K"), ("2", "A")]),
    "LED": ("D", [("1", "K"), ("2", "A")]),
    "Conn_Battery": ("J", [("1", "-"), ("2", "+")]),
    "SW_SPDT": ("SW", [("1", "A"), ("2", "COM"), ("3", "B")]),
    "Motor": ("M", [("1", "+"), ("2", "-")]),
    "SolderJumper": ("JP", [("1", "1"), ("2", "2")]),
    "MountingHole": ("H", []),
}


def symbol(name, prefix, pins):
    n = max(len(pins), 1)
    h = n * P
    lines = [
        "(kicad_symbol_lib",
        "\t(version 20251024)",
        '\t(generator "lite_gen_lib")',
        '\t(symbol "%s"' % name,
        "\t\t(pin_names (offset 1.016))",
        "\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)",
        '\t\t(property "Reference" "%s" (at 0 %s 0) (effects (font (size 1.27 1.27))))' % (prefix, _f(h / 2 + 2.54)),
        '\t\t(property "Value" "%s" (at 0 %s 0) (effects (font (size 1.27 1.27))))' % (name, _f(-h / 2 - 2.54)),
        '\t\t(property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
        '\t\t(property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
        '\t\t(symbol "%s_0_1"' % name,
        "\t\t\t(rectangle (start -5.08 %s) (end 5.08 %s) (stroke (width 0.254) (type default)) (fill (type background)))"
        % (_f(h / 2 + 1.27), _f(-h / 2 - 1.27)),
        "\t\t)",
        '\t\t(symbol "%s_1_1"' % name,
    ]
    for i, (num, pname) in enumerate(pins):
        y = h / 2 - P / 2 - i * P
        kind = "no_connect" if pname == "NC" else "passive"
        lines.append('\t\t\t(pin %s line (at -7.62 %s 0) (length 2.54) (name "%s" (effects (font (size 1.27 1.27)))) '
                     '(number "%s" (effects (font (size 1.27 1.27)))))' % (kind, _f(y), pname, num))
    lines += ["\t\t)", "\t)", ")"]
    with open(os.path.join(HERE, "symbols", name + ".kicad_sym"), "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    os.makedirs(os.path.join(HERE, "footprints"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "symbols"), exist_ok=True)
    for fn in (esp32_minikit, gy521, tp4056, jst_ph2, switch_spdt, cp_radial, led3, motor_pads, sot23, sot23_5,
               r1206, diode_combo, solder_jumper, mounting_hole):
        fn()
    r1206("C_1206_Mill")
    for name, (prefix, pins) in SYMBOLS.items():
        symbol(name, prefix, pins)


if __name__ == "__main__":
    main()

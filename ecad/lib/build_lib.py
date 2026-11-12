#!/usr/bin/env python3
"""Regenerate ecad/lib/{symbols,footprints} (provenance record; output is committed).

Sources
  KSYM  sparse clone of https://gitlab.com/kicad/libraries/kicad-symbols.git
  KFP   sparse clone of https://gitlab.com/kicad/libraries/kicad-footprints.git
  STD   pcb toolchain stdlib (bundled KiCad subset)

Three kinds of symbol:
  copied   verbatim from KiCad (derived symbols get their `extends` parent in the
           same file, which is what pcb needs to resolve them)
  adapted  a KiCad symbol of a pin-compatible part, renamed; pinout re-checked
           against the datasheet named in PINOUT_EVIDENCE
  authored generated here from the pin table in PINOUT_EVIDENCE

Usage: python3 lib/build_lib.py [--ksym DIR] [--kfp DIR] [--std DIR]
"""
import argparse
import os
import re
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))

# Where each non-KiCad pinout comes from. "datasheet" = read from the PDF text;
# "VERIFY" = the vendor site was unreachable from the build container and the
# pinout comes from a secondary source, so it must be checked before fab.
PINOUT_EVIDENCE = {
    "ICM-42688-P": "datasheet: TDK DS-000347 rev 1.2, Table 9 / Figure 4 (section 4.1)",
    "BMI270": "datasheet: Bosch BST-BMI270-DS000-06, Table 22 (section 7.1)",
    "LMR36015": "datasheet: TI SNVSB49D, Figure 7-1 and Pin Functions table",
    "LDL1117S33R": "datasheet: ST DS12022 (LDL1117), Table 1; tab = VOUT",
    "STM32H743VITx": "KiCad symbol; power pins re-checked against ST DS12110 rev 7 Table 9",
    "DPS310": "VERIFY: no datasheet reachable; pins from atopile/packages "
    "infineon-dps310 (EasyEDA C3232509)",
    "W25N01GVZEIG": "VERIFY: no datasheet reachable; KiCad W25Q128JVE (same Winbond "
    "WSON-8 8x6 pinout) cross-checked with the SamacSys W25N01GVZEIG symbol in pms67/HadesFCS",
}


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


def symbol_body(text):
    """Return the top-level (symbol ...) s-expression of a one-symbol library."""
    start = text.index('\t(symbol "')
    end = text.rstrip().rindex(")")
    return text[start:end].rstrip() + "\n"


HEADER = '(kicad_symbol_lib\n\t(version 20251024)\n\t(generator "kicad_symbol_editor")\n\t(generator_version "10.0")\n'


def lib_of(*bodies):
    return HEADER + "".join(bodies) + ")\n"


def set_prop(body, key, value):
    pat = re.compile(r'(\(property "%s" )"(?:[^"\\]|\\.)*"' % re.escape(key))
    assert pat.search(body), key
    return pat.sub(lambda m: m.group(1) + '"' + value.replace('"', '\\"') + '"', body, count=1)


def rename(body, old, new):
    return body.replace('(symbol "%s' % old, '(symbol "%s' % new)


def pin_sexpr(ptype, x, y, rot, name, number, hide=False):
    return (
        "\t\t\t(pin %s line\n\t\t\t\t(at %.2f %.2f %d)\n\t\t\t\t(length 2.54)\n%s"
        '\t\t\t\t(name "%s"\n\t\t\t\t\t(effects\n\t\t\t\t\t\t(font\n\t\t\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t\t\t)\n\t\t\t\t\t)\n\t\t\t\t)\n'
        '\t\t\t\t(number "%s"\n\t\t\t\t\t(effects\n\t\t\t\t\t\t(font\n\t\t\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t\t\t)\n\t\t\t\t\t)\n\t\t\t\t)\n\t\t\t)\n'
        % (ptype, x, y, rot, "\t\t\t\t(hide yes)\n" if hide else "", name, number)
    )


def prop_sexpr(key, value, y, hide=True):
    return (
        '\t\t(property "%s" "%s"\n\t\t\t(at 0 %.2f 0)\n%s\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
        % (key, value.replace('"', '\\"'), y, "\t\t\t(hide yes)\n" if hide else "")
    )


def author(name, footprint, datasheet, descr, left, right, evidence):
    """Generate a box symbol. left/right: lists of (number, pin name, electrical type).

    Several entries with the same pin name stack on one position (extra ones hidden),
    the KiCad convention for multi-pad power pins.
    """
    rows = max(len({p[1] for p in left}), len({p[1] for p in right}))
    half_h = 2.54 * (rows + 1) / 2
    half_w = 10.16
    body = '\t(symbol "%s"\n\t\t(exclude_from_sim no)\n\t\t(in_bom yes)\n\t\t(on_board yes)\n' % name
    body += prop_sexpr("Reference", "U", half_h + 2.54, hide=False)
    body += prop_sexpr("Value", name, half_h + 5.08, hide=False)
    body += prop_sexpr("Footprint", footprint, -half_h - 2.54)
    body += prop_sexpr("Datasheet", datasheet, -half_h - 5.08)
    body += prop_sexpr("Description", descr + " [pinout: " + evidence + "]", -half_h - 7.62)
    body += (
        '\t\t(symbol "%s_0_1"\n\t\t\t(rectangle\n\t\t\t\t(start %.2f %.2f)\n\t\t\t\t(end %.2f %.2f)\n'
        "\t\t\t\t(stroke\n\t\t\t\t\t(width 0.254)\n\t\t\t\t\t(type default)\n\t\t\t\t)\n"
        "\t\t\t\t(fill\n\t\t\t\t\t(type background)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n" % (name, -half_w, half_h, half_w, -half_h)
    )
    body += '\t\t(symbol "%s_1_1"\n' % name
    for side, x, rot in ((left, -half_w - 2.54, 0), (right, half_w + 2.54, 180)):
        seen = {}
        for number, pname, ptype in side:
            hide = pname in seen
            if not hide:
                seen[pname] = half_h - 2.54 * (len(seen) + 1)
            body += pin_sexpr("passive" if hide else ptype, x, seen[pname], rot, pname, number, hide)
    body += "\t\t)\n\t\t(embedded_fonts no)\n\t)\n"
    return lib_of(body)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ksym", default="/home/user/kicadlib/gsym")
    ap.add_argument("--kfp", default="/home/user/kicadlib/gfp")
    ap.add_argument("--std", default="/home/user/diodeinc/pcb/lib/std")
    a = ap.parse_args()
    sym_out = os.path.join(HERE, "symbols")
    fp_out = os.path.join(HERE, "footprints")

    def ksym(lib, name):
        return symbol_body(read(os.path.join(a.ksym, lib + ".kicad_symdir", name + ".kicad_sym")))

    # --- copied from KiCad (child first is fine: pcb resolves `extends` in-file) ---
    copied = {
        "USB_C_Receptacle_USB2.0_16P": [("Connector", "USB_C_Receptacle_USB2.0_16P")],
        "ICE40UP5K-SG48ITR": [("FPGA_Lattice", "ICE40UP5K-SG48ITR")],
        "Conn_ARM_JTAG_SWD_10": [("Connector", "Conn_ARM_JTAG_SWD_10")],
        "SW_Push": [("Switch", "SW_Push")],
        "Conn_01x04_MountingPin": [("Connector_Generic_MountingPin", "Conn_01x04_MountingPin")],
        "Conn_01x06_MountingPin": [("Connector_Generic_MountingPin", "Conn_01x06_MountingPin")],
        "Conn_01x08_MountingPin": [("Connector_Generic_MountingPin", "Conn_01x08_MountingPin")],
        "LP5907MFX-3.3": [("Regulator_Linear", "LP5907MFX-1.2"), ("Regulator_Linear", "LP5907MFX-3.3")],
        "TLV75512PDBV": [("Regulator_Linear", "TLV70012_SOT23-5"), ("Regulator_Linear", "TLV75512PDBV")],
        "USBLC6-2SC6": [("Power_Protection", "USBLC6-2P6"), ("Power_Protection", "USBLC6-2SC6")],
        "AO3400A": [("Transistor_FET", "Q_NMOS_GSD"), ("Transistor_FET", "AO3400A")],
        "Conn_ARM_SWD_TagConnect_TC2030-NL": [
            ("Connector", "Conn_ARM_SWD_TagConnect_TC2030"),
            ("Connector", "Conn_ARM_SWD_TagConnect_TC2030-NL"),
        ],
    }
    for out, parts in copied.items():
        write(os.path.join(sym_out, out + ".kicad_sym"), lib_of(*[ksym(*p) for p in parts]))

    # --- STM32H743VITx: KiCad symbol, with the two VCAP pins given distinct names.
    # Zener connects pins by name, and the two VCAP pins must be separate nets so
    # each gets its own 2.2 uF (DS12110 Table 25 / Figure 17) without a tie track.
    mcu = ksym("MCU_ST_STM32H7", "STM32H743VITx")
    for num, new in (("48", "VCAP_1"), ("73", "VCAP_2")):
        pat = re.compile(r'\(name "VCAP"((?:(?!\(name ).)*?\(number "%s")' % num, re.S)
        assert pat.search(mcu), num
        mcu = pat.sub(lambda m: '(name "%s"' % new + m.group(1), mcu, count=1)
    write(os.path.join(sym_out, "STM32H743VITx.kicad_sym"), lib_of(mcu))

    # --- adapted from pin-compatible KiCad parts ---
    icm = rename(ksym("Sensor_Motion", "IIM-42652"), "IIM-42652", "ICM-42688-P")
    icm = set_prop(icm, "Value", "ICM-42688-P")
    icm = set_prop(icm, "Footprint", "Package_LGA:Bosch_LGA-14_3x2.5mm_P0.5mm")
    icm = set_prop(icm, "Datasheet", "https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf")
    icm = set_prop(
        icm,
        "Description",
        "6-axis IMU, SPI/I2C/I3C, LGA-14 [adapted from KiCad IIM-42652; pinout: %s; pin 7 is RESV "
        "'connect to GND' and is stacked on GND]" % PINOUT_EVIDENCE["ICM-42688-P"],
    )
    write(os.path.join(sym_out, "ICM-42688-P.kicad_sym"), lib_of(icm))

    bmi = rename(ksym("Sensor_Motion", "BMI160"), "BMI160", "BMI270")
    bmi = set_prop(bmi, "Value", "BMI270")
    bmi = set_prop(bmi, "Datasheet", "https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf")
    bmi = set_prop(bmi, "Description", "6-axis IMU, SPI/I2C, LGA-14 [adapted from KiCad BMI160; pinout: %s]" % PINOUT_EVIDENCE["BMI270"])
    write(os.path.join(sym_out, "BMI270.kicad_sym"), lib_of(bmi))

    w25 = rename(ksym("Memory_Flash", "W25Q32JVZP"), "W25Q32JVZP", "W25N01GVZEIG")
    w25 = set_prop(w25, "Value", "W25N01GVZEIG")
    w25 = set_prop(w25, "Footprint", "Package_SON:WSON-8-1EP_8x6mm_P1.27mm_EP3.4x4.3mm")
    w25 = set_prop(w25, "Datasheet", "https://www.winbond.com/resource-files/W25N01GV%20Rev%20R%20070323.pdf")
    w25 = set_prop(w25, "Description", "1 Gbit SLC NAND, SPI/QSPI, WSON-8 8x6 mm [adapted from KiCad W25Q32JVZP; pinout: %s]" % PINOUT_EVIDENCE["W25N01GVZEIG"])
    write(os.path.join(sym_out, "W25N01GVZEIG.kicad_sym"), lib_of(w25))

    # --- authored ---
    write(
        os.path.join(sym_out, "LMR36015.kicad_sym"),
        author(
            "LMR36015",
            "Package_DFN_QFN:Texas_RNX0012C_VQFN-14-11-1EP_2x3mm_P0.5mm_EP0.25x1.825mm",
            "https://www.ti.com/lit/ds/symlink/lmr36015.pdf",
            "4.2-60 V, 1.5 A synchronous buck, adjustable, VQFN-HR-12 (RNX)",
            [("2", "VIN", "power_in"), ("10", "VIN", "power_in"), ("9", "EN", "input"), ("8", "PG", "open_collector"),
             ("5", "VCC", "power_out"), ("6", "AGND", "power_in"), ("1", "PGND", "power_in"), ("11", "PGND", "power_in")],
            [("4", "BOOT", "passive"), ("12", "SW", "output"), ("3", "NC", "free"), ("7", "FB", "input")],
            PINOUT_EVIDENCE["LMR36015"],
        ),
    )
    write(
        os.path.join(sym_out, "LDL1117S33R.kicad_sym"),
        author(
            "LDL1117S33R",
            "Package_TO_SOT_SMD:SOT-223-3_TabPin2",
            "https://www.st.com/resource/en/datasheet/ldl1117.pdf",
            "1.2 A LDO, 3.3 V fixed, ceramic-stable, SOT-223",
            [("3", "VIN", "power_in"), ("1", "GND", "power_in")],
            [("2", "VOUT", "power_out")],
            PINOUT_EVIDENCE["LDL1117S33R"],
        ),
    )
    write(
        os.path.join(sym_out, "DPS310.kicad_sym"),
        author(
            "DPS310",
            "Package_LGA:Bosch_LGA-8_2x2.5mm_P0.65mm_ClockwisePinNumbering",
            "https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_02-EN.pdf",
            "Barometric pressure sensor, I2C/SPI, LGA-8 2.0x2.5 mm",
            [("8", "VDD", "power_in"), ("6", "VDDIO", "power_in"), ("1", "GND", "power_in"), ("7", "GND", "power_in")],
            [("4", "SCK", "input"), ("3", "SDI", "bidirectional"), ("5", "SDO", "bidirectional"), ("2", "CSB", "input")],
            PINOUT_EVIDENCE["DPS310"],
        ),
    )

    # --- footprints: verbatim from KiCad, stdlib for the ones it already bundles ---
    kfp = [
        "Package_QFP.pretty/LQFP-100_14x14mm_P0.5mm",
        "Package_LGA.pretty/Bosch_LGA-14_3x2.5mm_P0.5mm",
        "Package_LGA.pretty/Bosch_LGA-8_2x2.5mm_P0.65mm_ClockwisePinNumbering",
        "Package_SON.pretty/WSON-8-1EP_8x6mm_P1.27mm_EP3.4x4.3mm",
        "Package_DFN_QFN.pretty/Texas_RNX0012C_VQFN-14-11-1EP_2x3mm_P0.5mm_EP0.25x1.825mm",
        "Package_DFN_QFN.pretty/QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm",
        "Package_TO_SOT_SMD.pretty/SOT-223-3_TabPin2",
        "Package_TO_SOT_SMD.pretty/SOT-23-5",
        "Package_TO_SOT_SMD.pretty/SOT-23-6",
        "Package_TO_SOT_SMD.pretty/SOT-23",
        "Connector_JST.pretty/JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal",
        "Connector_JST.pretty/JST_SH_SM06B-SRSS-TB_1x06-1MP_P1.00mm_Horizontal",
        "Connector_JST.pretty/JST_SH_SM08B-SRSS-TB_1x08-1MP_P1.00mm_Horizontal",
        "Connector_USB.pretty/USB_C_Receptacle_HRO_TYPE-C-31-M-12",
        "Button_Switch_SMD.pretty/SW_SPST_TS-1088-xR020",
        "Inductor_SMD.pretty/L_Sunlord_SWPA4030S",
        "Connector.pretty/Tag-Connect_TC2030-IDC-NL_2x03_P1.27mm_Vertical",
    ]
    for rel in kfp:
        src = os.path.join(a.kfp, rel + ".kicad_mod")
        os.makedirs(fp_out, exist_ok=True)
        shutil.copyfile(src, os.path.join(fp_out, os.path.basename(src)))
    std_fp = os.path.join(a.std, "kicad-footprints")
    shutil.copyfile(
        os.path.join(std_fp, "Connector_PinHeader_1.27mm.pretty/PinHeader_2x05_P1.27mm_Vertical_SMD.kicad_mod"),
        os.path.join(fp_out, "PinHeader_2x05_P1.27mm_Vertical_SMD.kicad_mod"),
    )
    std_sym = os.path.join(a.std, "kicad-symbols")
    shutil.copyfile(
        os.path.join(std_sym, "Mechanical.kicad_symdir/MountingHole.kicad_sym"),
        os.path.join(sym_out, "MountingHole.kicad_sym"),
    )
    for dev in ("L_Small", "D_Small", "D_Schottky_Small"):
        shutil.copyfile(
            os.path.join(std_sym, "Device.kicad_symdir/%s.kicad_sym" % dev),
            os.path.join(sym_out, dev + ".kicad_sym"),
        )
    for dio in ("D_SOD-123", "D_SOD-323"):
        shutil.copyfile(
            os.path.join(std_fp, "Diode_SMD.pretty/%s.kicad_mod" % dio),
            os.path.join(fp_out, dio + ".kicad_mod"),
        )

    # DESIGN.md section 1: 4.0 mm unplated holes for M3 silicone grommets. KiCad has
    # no 4.0 mm hole, so resize the M3 one; the 7 mm keep-out is the courtyard.
    mh = read(os.path.join(std_fp, "MountingHole.pretty/MountingHole_3.2mm_M3_DIN965.kicad_mod"))
    mh = mh.replace("MountingHole_3.2mm_M3_DIN965", "MountingHole_4.0mm_M3_Grommet")
    mh = mh.replace("Mounting Hole 3.2mm, M3, no annular", "Mounting hole 4.0 mm NPTH for M3 silicone grommet, 7 mm keep-out")
    mh = mh.replace("(size 3.2 3.2)", "(size 4 4)").replace("(drill 3.2)", "(drill 4)")
    mh = mh.replace("(end 2.8 0)", "(end 3.5 0)").replace("(end 3.05 0)", "(end 3.5 0)")
    assert "(drill 4)" in mh and mh.count("(end 3.5 0)") == 2
    write(os.path.join(fp_out, "MountingHole_4.0mm_M3_Grommet.kicad_mod"), mh)

    lic = "Symbols and footprints here are copied or derived from the KiCad libraries\n(https://gitlab.com/kicad/libraries), licensed CC-BY-SA 4.0 with the KiCad library\nexception. See build_lib.py for the source of each file.\n"
    write(os.path.join(HERE, "LICENSE-KiCad.txt"), lic)


if __name__ == "__main__":
    main()

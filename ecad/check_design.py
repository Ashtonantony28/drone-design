#!/usr/bin/env python3
"""Check the Zener flight-controller design against the neuroflight-fc design package.

Verify:  cd ecad && pytest -q check_design.py        (or: python3 check_design.py)

What is compared
  build      `pcb build -D warnings` must pass: no errors, no warnings
  netlist    `pcb build --netlist` (JSON on stdout; the flag is hidden in --help but
             is what pcb's own tests use) against hw/pinmap.csv and hw/DESIGN.md
  BOM        `pcb bom --offline -f json` against hw/bom.csv

Environment
  PCB        pcb binary (default: `pcb` on PATH, then ~/.local/bin/pcb)
  NF_FC_DIR  neuroflight-fc checkout (default: /home/user/neuroflight-fc)
"""
import collections
import csv
import functools
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD = os.path.join(HERE, "boards", "nf_fc", "nf_fc.zen")
NF_FC = os.environ.get("NF_FC_DIR", "/home/user/neuroflight-fc")
PCB = os.environ.get("PCB") or shutil.which("pcb") or os.path.expanduser("~/.local/bin/pcb")

# ---------------------------------------------------------------------------
# Documented differences from the design package
# ---------------------------------------------------------------------------

# pinmap.csv net name -> net name in the Zener netlist.
NET_RENAMES = {
    # KiCad pairs differential nets by the _P/_N suffix only (Zener naming rule)
    "USB_DP": "USB_P",
    "USB_DM": "USB_N",
    # Nets that never leave the MCU module carry its instance prefix
    "HSE_IN": "MCU.HSE_IN",
    "HSE_OUT": "MCU.HSE_OUT",
}

# bom.csv MPN -> MPN used, for parts that were substituted.
MPN_SUBSTITUTIONS = {
    "LMR36015FSC5RNXR": (
        "LMR36015FBRNXR",
        "TI SNVSB49D lists no fixed-5 V RNX part; FB = adjustable, 1 MHz, FPWM. "
        "Output set by RFBT 100k / RFBB 24.9k (datasheet Table 10-1).",
    ),
    "SWPA4030S4R7MT": (
        "SWPA4030S6R8MT",
        "4.7 uH gives ripple factor K = 0.57 at 25.2 V; SNVSB49D Eq. 4 wants 0.2-0.4 "
        "and Table 10-1 lists 6.8 uH for 1 MHz / 5 V. Same 4030 footprint.",
    ),
}

# MPN used -> (quantity here - quantity in bom.csv, reason). Anything not listed
# must match bom.csv exactly.
BOM_DIFFS = {
    "CL05B104KO5NNNC": (-8, "bom.csv 32 is an estimate ('1 per VDD pin + every IC'); the netlist needs 24: "
                            "MCU 5 VDD + VBAT + VDDA + NRST, baro 2, flash 1, IMU 3, USB ESD 1, buck boot 1, "
                            "current RC 1, FPGA 7 (DNP)"),
    "CL05A105KA5NQNC": (-5, "bom.csv 12 is an estimate; needed: VDDA 1, buck VCC 1, U7 in 1, U8 in/out 2, U11 in/out 2 (DNP)"),
    "CL10A106KP8NNNC": (-5, "bom.csv 8 is an estimate; needed: MCU bulk, U7 out, U8 out (DESIGN.md 4 and 5.1)"),
    "CL05A225MQ5NSNC": (+1, "ICM-42688-P VDD needs 2.2 uF next to its 100 nF (TDK DS-000347 Figure 5)"),
    "0402WGF1001TCE": (-1, "bom.csv 6; its own note lists 5 uses: VBAT divider low side, 3 LEDs, CUR RC"),
    "0402WGF1003TCE": (+1, "buck feedback RFBT 100k (adjustable LMR36015, see MPN_SUBSTITUTIONS)"),
    "19-217/GHC-YR1S2/3T": (-2, "bom.csv uses one green MPN '(or equiv)' for LED1-3; blue and red are separate parts"),
    # --- parts the schematic needs that bom.csv does not list ---
    "LTST-C193TBKT-5A": (+1, "blue STATUS LED (LED2)"),
    "LTST-C190KRKT": (+1, "red WARN LED (LED3)"),
    "0402WGF2492TCE": (+1, "buck feedback RFBB 24.9k"),
    "0402WGF1000TCE": (+6, "100 ohm series on the J3/J4/J5 UART lines (DESIGN.md 5.9)"),
    "0402WGF0000TCE": (+2, "0 ohm in the two IMU SCK lines (DESIGN.md 5.6: 22 ohm pads, 0 ohm first)"),
    "CL05B103KB5NNNC": (+1, "ICM-42688-P VDDIO 10 nF (TDK DS-000347 Figure 5)"),
    "CL05C200JB5NNNC": (+1, "buck feed-forward CFF 20 pF (SNVSB49D Table 10-1)"),
    "GRM188R72A224KAC4D": (+2, "220 nF HF bypass at each LMR36015 VIN/PGND pair (SNVSB49D 10.2.1.2.6)"),
}

# bom.csv rows that are not purchasable parts and so are not in the pcb BOM.
BOM_NOT_PARTS = {"BZ-PADS", "J6", "TP"}

# DESIGN.md / bom.csv designators that differ here.
REFDES_RENAMES = {
    "LED1": "D4",  # stdlib Led generic uses the D prefix
    "LED2": "D5",
    "LED3": "D6",
}

# DNP: bom.csv says U10, U11 and J7. Their support passives are DNP with them.
DNP_PARTS = {"U10", "U11", "J7"}
DNP_SUPPORT_PATHS = ("FPGA.", "PWR.C_U11_")

# Supply pin -> rail for every IC (symbol pin names).
SUPPLY_PINS = {
    "U1": {"VDD": "3V3_D", "VBAT": "3V3_D", "VDDA": "VDDA", "VREF+": "VDDA", "VSS": "GND", "VSSA": "GND",
           "VCAP_1": "MCU.VCAP_1", "VCAP_2": "MCU.VCAP_2"},
    "U2": {"VDD": "3V3_IMU", "VDDIO": "3V3_IMU", "GND": "GND"},
    "U3": {"VDD": "3V3_IMU", "VDDIO": "3V3_IMU", "GND": "GND", "GNDIO": "GND"},
    "U4": {"VDD": "3V3_D", "VDDIO": "3V3_D", "GND": "GND"},
    "U5": {"VCC": "3V3_D", "GND": "GND", "EP": "GND"},
    "U6": {"VIN": "VBAT", "EN": "VBAT", "PGND": "GND", "AGND": "GND", "VCC": "PWR.BUCK_VCC"},
    "U7": {"VIN": "5V", "VOUT": "3V3_D", "GND": "GND"},
    "U8": {"IN": "PWR.IMU_LDO_IN", "EN": "PWR.IMU_LDO_IN", "OUT": "3V3_IMU", "GND": "GND"},
    "U9": {"VBUS": "VBUS", "GND": "GND"},
    "U10": {"VCC": "1V2_FPGA", "VCCPLL": "1V2_FPGA", "VCCIO_0": "3V3_D", "VCCIO_2": "3V3_D",
            "SPI_VCCIO1": "3V3_D", "VPP_2V5": "3V3_D", "GND": "GND"},
    "U11": {"IN": "3V3_D", "EN": "3V3_D", "OUT": "1V2_FPGA", "GND": "GND"},
}

# MCU power pads, ST DS12110 rev 7 Table 9, LQFP100 column.
MCU_POWER_PADS = {
    "VDD": {"11", "27", "50", "75", "100"},
    "VSS": {"10", "26", "49", "74", "99"},
    "VBAT": {"6"},
    "VSSA": {"19"},
    "VREF+": {"20"},
    "VDDA": {"21"},
    "VCAP_1": {"48"},
    "VCAP_2": {"73"},
}

# Everything allowed on 3V3_IMU (DESIGN.md 3: "IMU island only. Nothing else on it").
IMU_RAIL_MEMBERS = {
    "U2", "U3",  # the IMUs
    "U8",  # LP5907 output
    "TP4",
}
IMU_RAIL_PASSIVE_PATHS = ("IMU.", "PWR.C_U8_OUT")

# Connector pinouts, DESIGN.md section 6 (pad -> net). *_CONN nets are the
# connector side of the 100 ohm series resistors.
CONNECTORS = {
    "J2": {"1": "VBAT", "2": "GND", "3": "ESC_CUR", "4": "ESC_TLM",
           "5": "ESC_M1", "6": "CONN.ESC_M2", "7": "CONN.ESC_M3", "8": "CONN.ESC_M4"},
    "J3": {"1": "5V", "2": "GND", "3": "CONN.RC_TX_CONN", "4": "CONN.RC_RX_CONN"},
    "J4": {"1": "5V", "2": "GND", "3": "CONN.FLOW_TX_CONN", "4": "CONN.FLOW_RX_CONN"},
    "J5": {"1": "5V", "2": "GND", "3": "CONN.AUX_TX_CONN", "4": "CONN.AUX_RX_CONN", "5": "AUX_SCL", "6": "AUX_SDA"},
    "J6": {"1": "3V3_D", "2": "SWDIO", "3": "NRST", "4": "SWCLK", "5": "GND", "6": "SWO"},
    # ARM Cortex-Debug 10-pin: 1 VTref, 2 SWDIO, 3/5/9 GND, 4 SWCLK, 6 SWO, 10 nRESET
    "J7": {"1": "3V3_D", "2": "SWDIO", "3": "GND", "4": "SWCLK", "5": "GND", "6": "SWO", "9": "GND", "10": "NRST"},
}
# Series element between an MCU-side net and a connector-side net: (net a, net b, value)
SERIES = [
    ("MOTOR0", "ESC_M1", "33"), ("MOTOR1", "CONN.ESC_M2", "33"),
    ("MOTOR2", "CONN.ESC_M3", "33"), ("MOTOR3", "CONN.ESC_M4", "33"),
    ("RX_TX", "CONN.RC_TX_CONN", "100"), ("RX_RX", "CONN.RC_RX_CONN", "100"),
    ("FLOW_TX", "CONN.FLOW_TX_CONN", "100"), ("FLOW_RX", "CONN.FLOW_RX_CONN", "100"),
    ("AUX_TX", "CONN.AUX_TX_CONN", "100"), ("AUX_RX", "CONN.AUX_RX_CONN", "100"),
    ("ESC_CUR", "CURR_SENSE", "1k"),
    ("IMUA_SCK", "IMU.IMUA_SCK_R", "0"), ("IMUB_SCK", "IMU.IMUB_SCK_R", "0"),
]
# Pull resistors named in hw/pinmap.csv notes: net -> (rail, value)
PULLS = {
    "IMUA_CS": ("3V3_IMU", "10k"), "BARO_SCL": ("3V3_D", "4.7k"), "BARO_SDA": ("3V3_D", "4.7k"),
    "QSPI_NCS": ("3V3_D", "10k"), "QSPI_IO2": ("3V3_D", "10k"), "QSPI_IO3": ("3V3_D", "10k"),
    "AUX_SCL": ("3V3_D", "2.2k"), "AUX_SDA": ("3V3_D", "2.2k"),
    "FPGA_SS_B": ("3V3_D", "10k"), "FPGA_CRESET_B": ("3V3_D", "10k"), "FPGA_IRQ": ("GND", "100k"),
    "BOOT0": ("GND", "10k"), "BEEPER": ("GND", "10k"),
}
# Test points, DESIGN.md section 8 (TP7 is "GND x2": TP7 and TP15; TP10 is after the series R).
TEST_POINTS = {
    "TP1": "VBAT", "TP2": "5V", "TP3": "3V3_D", "TP4": "3V3_IMU", "TP5": "1V2_FPGA", "TP6": "VDDA",
    "TP7": "GND", "TP15": "GND", "TP8": "PWR.BUCK_SW", "TP9": "VBAT_SENSE", "TP10": "ESC_M1",
    "TP11": "IMUA_INT1", "TP12": "IMUB_INT1", "TP13": "NRST", "TP14": "BOOT0",
}

# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

Component = collections.namedtuple("Component", "ref path attrs pins pads")
# pins: symbol pin name -> net name; pads: pad number -> net name


def _run(*args):
    return subprocess.run([PCB, *args], cwd=HERE, capture_output=True, text=True)


def _attr(v):
    return next(iter(v.values())) if isinstance(v, dict) and len(v) == 1 else v


@functools.lru_cache(maxsize=None)
def build():
    """`pcb build -D warnings`: (returncode, combined output)."""
    r = _run("build", "-D", "warnings", BOARD)
    return r.returncode, r.stdout + r.stderr


@functools.lru_cache(maxsize=None)
def netlist():
    r = _run("build", "--netlist", BOARD)
    assert r.returncode == 0, r.stderr
    data = json.loads(r.stdout)
    root = data["root_ref"] + "."
    port_net = {}
    nets = {}
    for name, net in data["nets"].items():
        ports = [p[len(root):] for p in net["ports"]]
        nets[name] = {"kind": net["kind"], "ports": ports}
        for p in ports:
            assert p not in port_net, "port %s is on two nets" % p
            port_net[p] = name
    comps = {}
    for key, inst in data["instances"].items():
        if inst.get("kind") != "Component":
            continue
        path = key[len(root):]
        pins, pads = {}, {}
        for pin, child in inst["children"].items():
            net = port_net.get(path + "." + pin)
            pins[pin] = net
            for pad in _attr(data["instances"][child]["attributes"]["pads"]):
                pads[_attr(pad)] = net
        ref = inst["reference_designator"]
        assert ref not in comps, "duplicate designator " + ref
        comps[ref] = Component(ref, path, {k: _attr(v) for k, v in inst["attributes"].items() if not k.startswith("__")}, pins, pads)
    return nets, comps


@functools.lru_cache(maxsize=None)
def bom():
    r = _run("bom", "--offline", BOARD, "-f", "json")
    assert r.returncode == 0, r.stderr
    return [m for group in json.loads(r.stdout) for m in group["members"]]


def read_csv(name):
    with open(os.path.join(NF_FC, "hw", name), newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def two_pin(kind, net_a, net_b):
    """Components of `kind` (attrs type) connecting exactly net_a and net_b."""
    _, comps = netlist()
    return [c for c in comps.values() if c.attrs.get("type") == kind and sorted(c.pins.values()) == sorted([net_a, net_b])]


def value_is(comp, want):
    """Compare the leading value token, e.g. '10k 1%' with '10k', '100nF 16V X7R' with '100nF'."""
    return comp.attrs["value"].split()[0] == want


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_build_clean():
    rc, out = build()
    assert rc == 0, out
    assert "warning" not in out.lower() and "error" not in out.lower(), out


def test_a_pinmap():
    """(a) every pinmap.csv row: package pin -> MCU pin name -> net."""
    _, comps = netlist()
    mcu = comps["U1"]
    assert mcu.attrs["mpn"] == "STM32H743VIT6"
    rows = read_csv("pinmap.csv")
    assert len(rows) >= 50
    used = set()
    for row in rows:
        pad, pin, want = row["lqfp_pin"], row["pin"], NET_RENAMES.get(row["net"], row["net"])
        assert mcu.pads[pad] == want, "pin %s (%s): expected %s, got %s" % (pad, pin, want, mcu.pads[pad])
        assert mcu.pins[pin] == want, "MCU pin name %s is not on %s" % (pin, want)
        assert len(netlist()[0][want]["ports"]) >= 2, "%s goes nowhere" % want
        used.add(pad)
    assert len(used) == len(rows), "pinmap.csv repeats a package pin"
    # every other signal pin is a spare (DESIGN.md section 7) and must be open
    nets, _ = netlist()
    for pad, net in mcu.pads.items():
        if pad in used or any(pad in pads for pads in MCU_POWER_PADS.values()):
            continue
        assert nets[net]["kind"] == "NotConnected", "MCU pad %s is on %s but is not in pinmap.csv" % (pad, net)


def test_b_mcu_decoupling():
    """(b) DESIGN.md 5.1: 100 nF per VDD pin, 10 uF bulk, 2.2 uF per VCAP, VDDA via FB2 + 1 uF + 100 nF."""
    nets, comps = netlist()
    mcu = comps["U1"]
    for name, pads in MCU_POWER_PADS.items():
        got = {pad for pad, net in mcu.pads.items() if net == mcu.pins[name] and pad in pads}
        assert got == pads, "%s pads %s" % (name, sorted(pads - got))
    mcu_caps = [c for c in comps.values() if c.path.startswith("MCU.") and c.attrs.get("type") == "capacitor"]
    on = lambda net, val: [c for c in mcu_caps if sorted(c.pins.values()) == sorted([net, "GND"]) and value_is(c, val)]
    n_vdd = len(MCU_POWER_PADS["VDD"]) + len(MCU_POWER_PADS["VBAT"])
    assert len(on("3V3_D", "100nF")) == n_vdd, "need one 100 nF per VDD/VBAT pin"
    assert len(on("3V3_D", "10uF")) >= 1, "no 10 uF bulk at the MCU"
    for vcap in ("MCU.VCAP_1", "MCU.VCAP_2"):
        assert len(on(vcap, "2.2uF")) == 1, vcap
        assert len(nets[vcap]["ports"]) == 2, "%s must only reach its capacitor" % vcap
    assert mcu.pins["VCAP_1"] != mcu.pins["VCAP_2"]
    assert len(on("VDDA", "1uF")) == 1 and len(on("VDDA", "100nF")) == 1
    fb = two_pin("ferrite_bead", "3V3_D", "VDDA")
    assert [c.ref for c in fb] == ["FB2"]
    # nothing but the MCU, its filter and TP6 on VDDA
    assert {p.split(".")[0] for p in nets["VDDA"]["ports"]} == {"MCU", "TP6"}
    # HSE: crystal + two equal load caps; NRST 100 nF
    y1 = comps["Y1"]
    assert {"MCU.HSE_IN", "MCU.HSE_OUT", "GND"} == set(y1.pins.values())
    assert len(on("MCU.HSE_IN", "12pF")) == 1 and len(on("MCU.HSE_OUT", "12pF")) == 1
    assert len(on("NRST", "100nF")) == 1


def test_c_no_single_pin_nets():
    """(c) a net with one pin must be an explicit no-connect; a no-connect has exactly one pin."""
    nets, comps = netlist()
    for name, net in nets.items():
        if net["kind"] == "NotConnected":
            assert len(net["ports"]) == 1, "%s is NotConnected but has %d pins" % (name, len(net["ports"]))
        else:
            assert len(net["ports"]) >= 2, "single-pin net %s (%s)" % (name, net["ports"])
    # and no component pin is left off the netlist
    for c in comps.values():
        for pin, net in c.pins.items():
            assert net is not None, "%s.%s is not on any net" % (c.ref, pin)
    # test-point-only stubs are deliberate: list them so a new one is noticed
    by_path = {c.path: c for c in comps.values()}
    stubs = set()
    for name, net in nets.items():
        owners = [by_path[p.rsplit(".", 1)[0]].ref for p in net["ports"]]
        if len(owners) == 2 and any(o.startswith("TP") for o in owners):
            stubs.add(name)
    assert stubs == {"IMU.IMUA_CLKIN", "LED_STRIP"}, stubs


def test_d_supply_rails():
    """(d) every IC supply pin is on its rail; 3V3_IMU feeds the IMU island only."""
    nets, comps = netlist()
    for ref, pins in SUPPLY_PINS.items():
        for pin, rail in pins.items():
            assert comps[ref].pins[pin] == rail, "%s.%s: expected %s, got %s" % (ref, pin, rail, comps[ref].pins[pin])
    ics = {ref for ref in comps if re.fullmatch(r"U\d+", ref)}
    assert ics == set(SUPPLY_PINS), "ICs without a supply check: %s" % sorted(ics ^ set(SUPPLY_PINS))
    by_path = {c.path: c for c in comps.values()}
    for port in nets["3V3_IMU"]["ports"]:
        c = by_path[port.rsplit(".", 1)[0]]
        assert c.ref in IMU_RAIL_MEMBERS or c.path.startswith(IMU_RAIL_PASSIVE_PATHS), "%s (%s) is on 3V3_IMU" % (c.ref, c.path)
    for ref in ("U2", "U3"):
        rails = {n for n in comps[ref].pins.values() if nets[n]["kind"] == "Power"}
        assert rails == {"3V3_IMU"}, "%s touches %s" % (ref, rails)
    # rails carry the voltages of DESIGN.md section 3 and each has one source
    assert nets["GND"]["kind"] == "Ground"
    for rail in ("VBAT", "VBUS", "5V", "3V3_D", "3V3_IMU", "1V2_FPGA", "VDDA"):
        assert nets[rail]["kind"] == "Power", rail
    # power tree links
    assert [c.ref for c in two_pin("ferrite_bead", "5V", "PWR.IMU_LDO_IN")] == ["FB1"]
    assert comps["D1"].pins == {"A": "VBUS", "K": "5V"}
    assert comps["D3"].pins == {"A": "GND", "K": "VBAT"} and comps["D3"].attrs["mpn"] == "SMBJ26A"
    assert comps["L1"].pins == {"1": "PWR.BUCK_SW", "2": "5V"}
    assert comps["U6"].pins["SW"] == "PWR.BUCK_SW" and comps["U6"].pins["FB"] == "PWR.BUCK_FB"


def test_e_bom():
    """(e) BOM count per MPN and DNP flags against hw/bom.csv."""
    members = bom()
    mine = collections.Counter(m["mpn"] for m in members)
    assert all(m.get("mpn") for m in members), "BOM line without an MPN"
    want = collections.Counter()
    ref_of = {}
    dnp_want = set()
    for row in read_csv("bom.csv"):
        if row["ref"] in BOM_NOT_PARTS:
            continue
        mpn = row["mpn"].replace(" (or equiv)", "")
        mpn = MPN_SUBSTITUTIONS.get(mpn, (mpn,))[0]
        want[mpn] += int(row["qty"])
        if re.fullmatch(r"[A-Z]+\d+", row["ref"]):
            ref_of[row["ref"]] = mpn
        if row["populate"] == "no":
            dnp_want.add(row["ref"])
    for mpn in sorted(set(mine) | set(want)):
        delta = BOM_DIFFS.get(mpn, (0,))[0]
        assert mine[mpn] == want[mpn] + delta, "%s: %d here, %d in bom.csv, documented difference %+d" % (mpn, mine[mpn], want[mpn], delta)
    assert set(BOM_DIFFS) <= set(mine), "stale BOM_DIFFS entries: %s" % sorted(set(BOM_DIFFS) - set(mine))
    # designators of single-reference rows carry the same part
    by_ref = {m["designator"]: m for m in members}
    for ref, mpn in ref_of.items():
        assert by_ref[REFDES_RENAMES.get(ref, ref)]["mpn"] == mpn, ref
    # DNP flags
    assert dnp_want == DNP_PARTS
    dnp = {m["designator"] for m in members if m.get("dnp")}
    assert DNP_PARTS <= dnp
    for m in members:
        if m.get("dnp") and m["designator"] not in DNP_PARTS:
            assert m["path"].startswith(DNP_SUPPORT_PATHS), "%s (%s) is DNP" % (m["designator"], m["path"])
            assert m.get("component_type") in ("Capacitor", "Resistor")
    # not-a-part rows: present on the board, absent from the BOM
    _, comps = netlist()
    assert "J6" in comps and "J6" not in by_ref
    assert sum(1 for r in comps if r.startswith("TP")) >= 14


def test_connectors_and_series():
    """DESIGN.md section 6 pinouts; INTERFACES.md section 2 motor order (MOTOR0-3 -> ESC M1-M4)."""
    _, comps = netlist()
    for ref, pads in CONNECTORS.items():
        for pad, net in pads.items():
            assert comps[ref].pads[pad] == net, "%s pad %s: expected %s, got %s" % (ref, pad, net, comps[ref].pads[pad])
    for a, b, val in SERIES:
        r = two_pin("resistor", a, b)
        assert len(r) == 1 and value_is(r[0], val), "%s -> %s: %s" % (a, b, [c.attrs["value"] for c in r])
    for net, (rail, val) in PULLS.items():
        r = two_pin("resistor", net, rail)
        assert len(r) == 1 and value_is(r[0], val), "%s pull to %s: %s" % (net, rail, [c.attrs["value"] for c in r])
    # bidirectional DShot: nothing on the motor nets but the MCU pin and the series resistor
    nets, _ = netlist()
    for i in range(4):
        assert len(nets["MOTOR%d" % i]["ports"]) == 2
    # USB: both data lines pass the ESD part, CC pins have Rd
    j1, u9, mcu = comps["J1"], comps["U9"], comps["U1"]
    assert j1.pins["D+"] == u9.pins["I/O1"] == mcu.pins["PA12"] == "USB_P"
    assert j1.pins["D-"] == u9.pins["I/O2"] == mcu.pins["PA11"] == "USB_N"
    for cc in ("CC1", "CC2"):
        r = two_pin("resistor", j1.pins[cc], "GND")
        assert len(r) == 1 and value_is(r[0], "5.1k")
    # dividers: VBAT 10k/1k, 5V 10k/10k, VBUS 10k/20k
    for top, mid, r_top, r_bot in (("VBAT", "VBAT_SENSE", "10k", "1k"), ("5V", "5V_SENSE", "10k", "10k"), ("VBUS", "VBUS_SENSE", "10k", "20k")):
        a, b = two_pin("resistor", top, mid), two_pin("resistor", mid, "GND")
        assert len(a) == 1 and value_is(a[0], r_top) and len(b) == 1 and value_is(b[0], r_bot), mid


def test_test_points_and_mounting():
    _, comps = netlist()
    for ref, net in TEST_POINTS.items():
        assert comps[ref].pins == {"1": net}, "%s: %s" % (ref, comps[ref].pins)
    holes = [c for r, c in comps.items() if r.startswith("H")]
    assert len(holes) == 4 and all("MountingHole_4.0mm" in c.attrs["footprint"] for c in holes)


def test_library_is_local():
    """Every non-stdlib symbol and footprint is a file under ecad/lib."""
    _, comps = netlist()
    for c in comps.values():
        for key in ("symbol_path", "footprint"):
            uri = c.attrs[key]
            assert uri.startswith(("package://workspace/lib/", "package://stdlib/")), "%s %s" % (c.ref, uri)
            if uri.startswith("package://workspace/"):
                assert os.path.isfile(os.path.join(HERE, uri[len("package://workspace/"):])), uri


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("PASS", name)
            except AssertionError as e:
                failed += 1
                print("FAIL", name, "-", e)
    sys.exit(1 if failed else 0)

# Lite FC: milling and hand assembly

Flight controller for the sub-$50 brushed micro quad (8520 coreless motors, 1S LiPo).
Board: 46 x 38 x 1.6 mm, 3 mm corner radius, M2 holes at (+-19, +-15) mm.
Double-sided copper-clad blank (FR-1 or FR-4), no plating.

Coordinates in this file are millimetres from the board centre, seen from the top:
x to the right (the nose, +x), y down (drone right, +y). Add (100, 100) for KiCad sheet coordinates.

Regenerate everything from `lite/ecad/`:

```
pcb build boards/lite_fc/lite_fc.zen        # Zener design -> netlist (39 parts)
layout/run.sh all                            # board, placement, routing, DRC, pictures
layout/run.sh fab                            # fab/: Gerbers, Excellon, SVG/PNG
pytest -q check_board.py                     # pin map, motor switches, mill rules, outline
```

## Mill rules

| Rule | Value | Where it is enforced |
|---|---|---|
| Layers | 2, unplated; layer change only through a through-hole pad or a 1.0 mm wire via | `.kicad_dru`, router |
| Signal track | >= 0.5 mm | `.kicad_pro`, router |
| Battery / motor track (VBAT, GND, BAT_P, BAT_N, M1_N..M4_N) | >= 1.0 mm. Exception: 0.5 mm stubs between the 0.95 mm pins of U4 | netclass Power, `.kicad_dru` |
| Clearance | >= 0.4 mm (the router keeps 0.45 mm) | `.kicad_pro` |
| Drill | >= 0.8 mm | `.kicad_pro` |
| Copper to edge / hole | >= 0.5 mm | `.kicad_pro` |
| SMD packages | SOT-23, SOT-23-5, SOD-123, 1206 only, all on the bottom | `check_board.py` |
| Top copper | VBAT and GND only | `.kicad_dru` |

KiCad DRC with these rules: 0 errors, 0 unconnected items. The only warnings are about silkscreen and four
track-to-pad necks that are a few hundredths of a millimetre under 0.5 mm.

## Which side to mill

**Bottom (B.Cu) is the main side.** It carries every signal, all the SMD parts, and a GND pour. The pour
connects SMD pads solid and through-hole pads with 0.6 mm spokes.
**Top (F.Cu) carries only 7 short VBAT/GND tracks** (listed below) and the through-hole pad rings.

1. Fix the blank and drill every hole from one side, using the same absolute origin for both files:
   `fab/drill/lite_fc-PTH.drl` and `fab/drill/lite_fc-NPTH.drl`.
2. Mill the bottom isolation using `fab/gerbers/lite_fc-B_Cu.gbr`. It is drawn as seen from the top, so
   mirror it in your CAM. `fab/lite_fc-bottom-mill.svg` is already mirrored. The narrowest gap is
   0.4 mm, so use a 0.2-0.3 mm V-bit or a 0.4 mm end mill.
3. Flip the board about its vertical centre line (x -> -x). Register it on pins through the four 2.2 mm
   holes, then mill the top using `fab/gerbers/lite_fc-F_Cu.gbr`. Rub out the free top copper, at
   least under the switch, the JST and the header bodies.
4. Cut the outline last (`fab/gerbers/lite_fc-Edge_Cuts.gbr`).

### Drill sizes

| Drill | Count | Holes |
|---|---|---|
| 0.8 mm | 4 | C1 (2), D5 (2) |
| 0.9 mm | 2 | J1 |
| 1.0 mm | 40 | U1 headers (13), U2 header (8), U3 wire legs (3), SW1 (3), motor pads M1-M4 (8), wire vias (5) |
| 2.2 mm NPTH | 4 | H1-H4 mounting holes (also the flip registration holes) |

## Layer changes

top-side tracks: 7
wire vias: 5

A layer change happens only where a through-hole pad is soldered on **both** sides, or at a wire via.
Pads under a plastic body (the ESP32 and GY-521 headers, SW1, J1) are never used for a layer change.

**Pads to solder on both sides:** M1+ , M3+ , M4+ (motor pads, VBAT), C1 - (GND), D5 cathode (GND), and
U3 OUT- (GND, the TP4056 wire leg). Leave about 2 mm of lead on C1 and D5 so that the iron reaches the
top joint.

**Wire vias.** Drill 1.0 mm, push a 0.6-0.8 mm tinned copper wire through, solder both sides, trim flush.

| Via | Net | At (x, y) | Joins |
|---|---|---|---|
| V1 | VBAT | (-8.6, -17.3) | top VBAT spine -> R9 (battery divider), C4, IMU VCC on the bottom |
| V2 | VBAT | (-8.1, 17.2) | SW1 throw B, LDO U4, C2 on the bottom -> top track T2 |
| V3 | GND | (-13.1, -12.2) | R7 / R10 ground island (top-left) -> top |
| V4 | GND | (-7.3, -14.9) | IMU GND, C4, R8, Q4 source on the bottom -> top |
| V5 | GND | (-0.8, 2.3) | U4 / C2 / C3 / ESP32 GND on the bottom -> top |

**Top-side tracks**

| Track | Net | Width | From -> to |
|---|---|---|---|
| T1 | VBAT | 1.0 mm | M1+ -> up the left side (x = -18.6) -> M3+ -> along the -y edge (y = -17.7, over V1) -> down at x = 16.6 -> M4+ (82 mm, the VBAT spine) |
| T2 | VBAT | 1.0 mm | V2 -> between SW1 and J1 -> M1+ |
| T3 | GND | 1.0 mm | D5 cathode -> TP4056 OUT- (U3.3) |
| T4 | GND | 1.0 mm | V5 -> U3.3 |
| T5 | GND | 1.0 mm | C1 - -> over the ESP32 GND pin (R7) -> V5 |
| T6 | GND | 1.0 mm | D5 cathode -> V3 |
| T7 | GND | 1.0 mm | V3 -> V4 |

These layer changes are needed because the ESP32 sits in the middle and its four PWM lines, I2C, LED, INT,
3V3 and battery-sense lines fan out to all four sides. On one layer, VBAT (all four corners) and GND
(every motor switch) would have to cross them. The router (`layout/router.py`) keeps signals on the
bottom. It pays 2.5x per millimetre of top track and the cost of 4 cm of track per wire via, so it uses
top copper only where the bottom is closed.

## Hand-assembly order

Work bottom first, low parts before tall ones. Trim every through-hole tail **flush** on the bottom: the
battery sits 3 mm under the board, about 1.4 mm below the tallest bottom part (`lite/spec.yaml`).

1. **Wire vias V1-V5.**
2. **Bottom SMD, smallest first:**
   - U4 (ME6211, SOT-23-5). Its CE pin is tied to VIN under the body, so check that bridge with a meter.
   - Q1-Q4 (AO3400A, SOT-23).
   - D1-D4 (B5819W, SOD-123). The cathode band goes towards the motor + pad.
   - R1-R11 and C2-C4 (1206).
   - Leave JP1 open.
3. **Check for shorts:** VBAT-GND, 3V3-GND and BAT_P-BAT_N must all be open.
4. **D5 LED** (cathode = square pad, towards -y) and **C1** (470 uF, + = square pad, towards the edge).
   Solder each on both sides of the board.
5. **J1** (PH2.0) and **SW1**.
6. **TP4056 module (U3):** solder short wire legs into B+, B- and OUT-, then solder the module flat on
   them, USB towards the nose. Solder OUT- on the top of the board too. Hold the USB end down with foam
   tape.
7. **GY-521 header** (VCC at the -y end, board body towards +x).
8. **Female headers** for the ESP32 MiniKit: a 1x7 on the left inner rows 2-8 and a 1x6 on the right
   inner rows 3-8. Solder male pins on the module's two inner rows only. The module then sits above
   the GY-521 and the TP4056.
9. **Before the ESP32 goes in:**
   - Battery on, switch on. Check VBAT at C1 and 3.3 V on the ESP32 3V3 pin (L8).
   - With the switch off, VBAT must be 0 V.
10. **Motors:** red lead to the square + pad. Solder M1+, M3+ and M4+ on both sides; they carry the top
    VBAT track.
    - M1 rear-right
    - M2 front-right
    - M3 rear-left
    - M4 front-left

## Pin map (ESP32 MiniKit inner rows)

| Pad | GPIO | Net | Use |
|---|---|---|---|
| L2 | 36 (ADC1_CH0, input only) | VBAT_SENSE | 100k / 47k battery divider (4.2 V -> 1.34 V) |
| L3 | 26 | M3_PWM | rear-left motor |
| L4 | 18 | LED | status LED D5 (330R) |
| L5 | 19 | M1_PWM | rear-right motor |
| L6 | 23 | IMU_INT | MPU-6050 INT |
| L7 | 5 (strapping) | - | left free |
| L8 | 3V3 | 3V3 | fed by U4 (ME6211) |
| R3 | 22 | I2C_SCL | MPU-6050 |
| R4 | 21 | I2C_SDA | MPU-6050 |
| R5 | 17 | M4_PWM | front-left motor |
| R6 | 16 | M2_PWM | front-right motor |
| R7 | GND | GND | |
| R8 | 5V | - | unconnected (live only on USB) |

No motor is on a strapping pin (0, 2, 5, 12, 15) or an input-only pin (34-39). All four motor pins have
LEDC PWM.

## Power path notes

- **Charger and switch:** battery (J1) -> TP4056 B+/B-. B+ is the same copper as the module's OUT+. It
  feeds SW1 COM, and SW1 throw B is VBAT, so the switch is on the high side. The module's OUT- (behind
  the DW01 protection FETs) is board GND.
- **Protection trip:** the DW01/8205A protection on these modules trips at about 3 A (VERIFY). Four 8520
  motors can pull more than that at full throttle. Bridging JP1 (bottom, next to the TP4056 pads)
  bypasses the protection FETs, and with them the over-discharge cut-off. If you bridge it, use a pack
  with its own protection, or watch VBAT_SENSE.
- **Switch rating:** the SS-12D00 is rated about 0.5 A (VERIFY), well under the motor current. It works
  for short flights, but its contacts heat up and drop voltage. For hard flying, fit a higher-current
  switch on the same 2.54 mm pads, or bridge COM to B and unplug the battery to switch off.
- **ESP32 3V3:** the ESP32 is powered through its 3V3 pin, which bypasses the module's own regulator. With
  USB plugged in as well, the two 3.3 V regulators run in parallel. That is harmless for the module's
  LDO, but leave the battery off while flashing if in doubt.
- **IMU supply:** the GY-521 is fed from VBAT through its own 3.3 V LDO (and its I2C pull-ups sit behind
  that LDO). Below about 3.4 V the LDO drops out and the MPU-6050 sees VBAT minus 0.1-0.2 V, which is
  still inside its 2.375-3.46 V range (VERIFY the breakout's LDO).

## Mechanical notes

- **TP4056 module:** occupies a band along the +y edge (y 3.8-21.3). It overhangs the drone-right edge by
  2.3 mm and must be no longer than 26.0 mm. Its x range (-8.8 to 17.2) leaves 0.05 mm to SW1 and to the
  3.5 mm head of the H3 screw. Use button-head (ISO 7380) M2 screws at H3 and H4; a 3.8 mm cheese head
  touches SW1 and the module.
- **Height under the board:** the bottom SMD parts are 1.1 mm tall at most (SOD-123, SOT-23). An SMA
  diode (2.3 mm) does not fit above the battery.
- **ESP32 module:** overhangs the -y edge with its antenna. It sits 8.5 mm up on its female headers,
  above the GY-521 and the TP4056. The stack stays within the 14 mm avionics height of `lite/spec.yaml`
  as long as the headers are the 8.5 mm type.
- **IMU position:** the GY-521 body centre is at about (3, -6.6), 7 mm off the board centre towards -y,
  because the TP4056 band takes the +y half. Its axes are aligned with the frame: x to the nose.

## VERIFY before building

1. MiniKit pinout and geometry: inner rows 22.86 mm apart, row 1 about 7.7 mm from the antenna end, and
   the pin order of both inner columns (RIOT-OS / MH-ET LIVE drawing).
2. GY-521 outline (20.3 x 15.6 mm), pin order VCC GND SCL SDA XDA XCL AD0 INT, AD0 pull-down on the
   breakout (address 0x68), and the dropout of the breakout's LDO.
3. TP4056 module: length 26.0 mm or less, pad order OUT- / B- / B+ along the battery end, and that B+
   is the same copper as OUT+. Also its protection trip current (about 3 A).
4. JST PH2.0 polarity of your battery packs against the +/- marking.
5. SS-12D00 lead pitch (2.54 mm assumed; some versions are 2.0 mm) and its current rating.
6. Order codes of the generic parts in `bom.csv` (UniOhm / Samsung / CJ); any equivalent will do.

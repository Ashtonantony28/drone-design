# drone-design: NF-3D

A complete small quadcopter designed entirely in code, in a cloud container. It is built around the
[neuroflight-fc](https://github.com/Ashtonantony28/neuroflight-fc) learned flight controller and
flight-tested in that project's simulator.

- **Electronics:** the neuroflight-fc flight controller written in **Zener** with Diode's
  [`pcb`](https://github.com/diodeinc/pcb) toolchain (the E-CAD used in the Fable 5.2 electronics demo),
  laid out and DRC-checked in **KiCad 10**.
- **Mechanics:** a parametric frame in **build123d** (Python CAD), with mass, clearance, FEA and
  manufacturing exports.
- **Validation:** propulsion model, structural FEA and flight simulation, each with pass/fail gates.

![iso render](out/cad/render_iso.png)

## The vehicle

| | |
|---|---|
| Class | 3" ducted quad, 4S, sub-250 g |
| Motor spacing | 100 mm square (141 mm diagonal), widened from neuroflight's 92 mm so the 30.5 mm FC stack clears the ducts |
| All-up weight | **238.8 g** (CAD mass model, requirement ≤ 249 g) |
| Thrust/weight | **5.9** nominal, 4.3 in the pessimistic corner (requirement ≥ 3) |
| Hover | 28 % throttle, **8.8 min** hover (5.3 min pessimistic; requirement ≥ 4) |
| Frame | 3.0 mm carbon unibody with closed ring perimeter, PA12-CF printed ducts, TPU feet |
| FC | STM32H743, ICM-42688-P + BMI270, DPS310, 1 Gbit NAND, 36x36 mm 4-layer, 143 parts |

Every number comes from `spec/vehicle.yaml`. It is the single source that the CAD, mass model,
propulsion model, FEA and flight simulator all read.

## Verify

```
make verify          # mech -> propulsion -> fea -> flight (needs ../neuroflight-fc, torch)
cd ecad && python -m pytest check_design.py check_layout.py -q
docker run --rm -v $PWD:$PWD -w $PWD/ecad kicad/kicad:10.0.8 \
  kicad-cli pcb drc --severity-all boards/nf_fc/layout/nf_fc/nf_fc.kicad_pcb
```

| Check | Result |
|---|---|
| `mech.checks`: interference, clearances ≥ 1 mm, prop tip gap 1.0 mm, prop inside duct, feet are the lowest point, CoG offset | PASS |
| `prop.propulsion`, nominal and pessimistic corners (ct −15 %, cp +20 %, mass +5 %) | PASS |
| `mech.fea`: 20 g single-foot crash | SF **1.53** (≥ 1.5) at 1.6 mm mesh |
| `mech.fea`: full thrust | SF 31 |
| `mech.fea`: first frame mode | 213-221 Hz, 21-32 % away from hover rotor speed (≥ 20 %) |
| `flight.fly`: PID fallback on NF-3D vs the original neuroflight vehicle, 256 randomized vehicles per scenario | PASS (within 3 points in both corners) |
| `flight.fly`: v3 policy with thrust calibration (`v3cal`), same comparison (advisory) | within 3 points in both corners |
| `pcb build` (Zener) | 143 components, no errors or warnings |
| `ecad/check_design.py` (pin map, decoupling, rails, BOM vs neuroflight-fc) | 9 passed |
| `ecad/check_layout.py` (IMU spacing, DShot length, holes, decoupling, GND plane, USB pair, QSPI match, power widths and pours, track/space margin, silkscreen) | 17 passed |
| KiCad 10 DRC | **0 errors, 0 unconnected, 0 warnings** (the two library-footprint comparison checks, 120 warnings that depend on the installed footprint libraries, are set to ignore in `layout/place.py`) |

The frame took five FEA iterations to pass: crash safety factor 0.5 → 0.95 → 1.27 → 1.45 → 1.7, and
first mode 118 → 221 Hz. The changes were ring bridges, a 3 mm plate, wider arms, a bigger centre body
and filleted arm roots.

## Findings and open items

1. **The neural policy flies NF-3D after a thrust calibration, with no retraining.** neuroflight's
   `hover_ctbr_v3` used as-is holds up on the nominal NF-3D (99.2/85.9/95.3 % hover/tilt/throw). On the
   weak-motor corner it drops to 50/34/54 %, because its thrust command is in the reference vehicle's
   units and the policy has no integral action. `flight/fly.py` `CollectiveScaled` reads the command as
   mass-normalized and rescales it to this airframe:
   `a0' = (a0+1)·s − 1`, with `s = (T_max,ref · m) / (T_max · m_ref)`. That is what a hover-thrust
   estimator does on PX4/ArduPilot. With it, v3 scores **100/89.5/98.0 %** nominal and
   **100/90.2/98.0 %** pessimistic, against 100/90.6/97.3 % on the reference vehicle, so both corners
   are within the 3-point margin. Fine-tuning was tried first and doesn't work: every run lost v3's
   cm-level hover, including on the reference vehicle. `flight/retrain.py` records what was ruled out.
   Firmware follow-up for neuroflight-fc: apply `s` (or a learned hover thrust) between the policy
   output and the mixer.
2. **PCB before fab:** the layout items are done and asserted in `ecad/check_layout.py`
   (`python check_layout.py` prints the numbers). USB D+/D− is one 0.24 mm / 0.155 mm pair on L1
   (90.3 Ω by `ecad/layout/impedance.py`), 20.19 mm per line with 0.000 mm skew for both plug
   orientations (was D+ 53 mm vs D− 21 mm); 12.4 mm of it is coupled, the rest is fan-out at the
   MCU pins and the connector. The six QSPI lines are within 1.79 mm (was 29 mm apart on this
   routing), at the price of length: 53.6–55.4 mm each, because the longest line sets the target.
   VBAT, 5V and VBUS tracks are 0.25–0.5 mm (0.25 mm only in necks: 6 of 38 mm, 16 of 101 mm,
   2 of 28 mm) with L4 pours at the buck input and output; 3V3_D is the L3 pour. Silkscreen and
   the dangling via are clean. Still open: 342 mm of 1867 mm of track (18 %) stays 0.1 mm wide
   and 206 mm (11 %) has 0.1 mm space, in the MCU and FPGA escapes; 5V, VBUS and VBAT still have
   0.3–0.5 mm stretches on the 17.5 µm L3 (23, 9 and 9 mm) that should move to the outer layers;
   and the 90 Ω figure is a calculation, not a fab-controlled impedance. The router is this
   repo's own maze router (`ecad/layout/autoroute.py`); another seed or placement change can
   leave a net open, so re-run `layout/run.sh all` and the checks after any edit.
3. **Datasheet VERIFY items:** DPS310 / W25N01GV pinouts, LMR36015 variant and land pattern, IMU
   LGA-14 land patterns, crystal load capacitors, and the J2 pinout against the ESC you buy. All are
   marked `VERIFY` in `ecad/`.
4. **Propulsion numbers are estimates** until the M3 thrust stand; the pessimistic corner covers
   them. The arm mode is 21 % from the nominal hover rotor speed. Re-check once real hover rpm is known.

## Layout

```
spec/vehicle.yaml   single source of truth (geometry, parts, materials, motor/prop, requirements)
mech/               build123d frame, checks, FEA (gmsh + scikit-fem), exports -> out/cad/
prop/               battery-ESC-motor-prop model
flight/             flight validation and policy fine-tune in the neuroflight-fc simulator
ecad/               Zener FC design, layout scripts, KiCad board, checks, fab outputs (gerbers, PnP, STEP)
out/                generated reports, CAD exports, vehicle parameter sets
```

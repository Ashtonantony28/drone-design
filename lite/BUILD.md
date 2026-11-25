# Building the NF-3D Lite

The parts and costs are in the BOM sheet (base build about $40-88, with the autonomy kit $69-144). The files are in `lite/out/cad` (frame) and `lite/ecad/fab` (board).

**v4 changes (design review, Nov 2026).** The battery now plugs in through a BT2.0 connector on a short pigtail, the slide
switch and the on-board TP4056 charger are gone (charge the pack with a separate 1S charger), the flyback diodes are
2 A parts, the props sit 1.7 mm higher, the prop guard is at rotor height, and the firmware lands the drone on a flat
battery and stops the motors if the IMU stops answering.

## 1. Test the motors first (before you print or mill anything)
Brushed 8520 motors vary from batch to batch. The model's nominal motor gives about 34 g of thrust at full power; the pessimistic corner (motors 15 % weaker, props 20 % hungrier) gives about 26 g, which is not enough for the 1.6 thrust-to-weight requirement, so the bench test below rejects it. `python -m lite.checks` prints the threshold for each build (`acceptance` in `lite/out/checks.json`).
1. Wire one motor and its prop to a charged 1S battery through the board, or straight to the battery for 2 s.
2. Tape the motor upside down on a kitchen scale, so the prop blows upward and pushes down on the scale. Read the grams at full power.
3. **Accept the motors only if every one reaches the threshold for your build, on a fully charged pack, and all four are within about 2 g of each other:**

   | Build | All-up weight | Each motor must show |
   |---|---|---|
   | Base (no autonomy kit, no guard) | about 66 g | **27 g** or more |
   | With the autonomy kit | about 77 g | **31 g** or more |
   | Kit and prop guard | about 84 g | **34 g** or more (only full-spec motors; otherwise leave the guard off) |

   These are 1.6 × the weight, shared by four motors. The stress sim (`python -m lite.stress`) shows why matching matters: on weak motors, one motor 15 % weaker than the rest drops recovery from a throw to about 92 %, and adding 15 % weight pushes hover to 88 % throttle, which leaves almost nothing for control. If you have a USB or clamp meter, also reject a motor that draws more than 3.5 A at full power. Return failing motors or buy a different batch. 1020 motors fit if you widen the motor tube (`motor_bore` in `lite/spec.yaml`).
4. Once the board is built, you can do the same test with the board's serial command `spin <motor> <duty>` (see below).

## 2. Print the frame
- `frame.stl`: **PETG** (v2), 0.2 mm layers, 3 walls, 100 % infill on the arms (30 % elsewhere is fine). Print it flat, as exported. It needs no supports. PETG is used because the motor cans get warm enough to soften PLA (PLA softens at about 55-60 °C, PETG at about 80 °C). PLA still works for a first test print.
- The print is about 11.3 g. If it comes out over 14 g, reduce the infill in the centre plate.
- **Motor clamps.** Each motor tube has a slit on its outboard side and a groove round it. Push the motor in until it sits on the ledge inside the tube, with its leads through the hole in the bottom, then pull a 2.5 mm zip tie tight in the groove. Print a test tube first if your printer runs tight or loose: `motor_bore` in `lite/spec.yaml` is the knob.
- **Battery.** It slides in from the front and stops against the two posts at the back. A strap goes through the two slots in the plate.
- **Optional prop guard** (`guard.stl`, PETG): it drops over the tops of the four motor tubes, and its rings stand on posts so that they surround the props at their own height (3 mm from the tips). Print it upside down, ring tops on the bed, with supports under the spokes and collars. On the base build it adds about 7 g; check `with_guard` in `lite/out/checks.json` for hover throttle and flight time. Fit it for your first indoor flights, on motors that pass the threshold above.

## 3. Make the board
Follow `lite/ecad/MILLING.md`. In short:
1. Mill the bottom side and drill the holes.
2. Make the 5 wire vias and the 7 top tracks.
3. Solder the parts, low ones first.
4. Trim all through-hole leads flush, because the battery sits under the board.

Two of the mounting screws (H3 and H4) must be **low button-head M2** screws.

**Power path (v4).** Hover draws about 3.7 A and full throttle about 13.6 A, which the old PH2.0 header (2 A), slide switch (0.5 A) and charger-module protection (about 3 A) could not carry. The board is unchanged; it is assembled differently:
- **Battery lead:** solder a BT2.0 pigtail (22 AWG, about 40 mm) straight into the J1 holes, red to the + hole, instead of the PH2.0 header. Lay the connector aft along the board's rear-right corner and fix the wires with a dot of hot glue. Use packs with a BT2.0 plug.
- **No switch:** leave SW1 out and solder a short 22 AWG link from its middle pad (COM) to the pad nearest the ESP32 (throw B, VBAT). Plugging in the battery turns the drone on.
- **No on-board charger:** leave the TP4056 module (U3) out and bridge JP1 with a short 22 AWG link on the bottom (keep it flat; the battery sits 1.5 mm below). Charge the pack off the drone with a 1S charger at 1C or less (600 mA for a 600 mAh pack). This also removes the old 1 A charge rate (1.7C) and the charger's protection, which tripped under hover current.
- **Thicken the battery tracks:** run a bead of solder along the VBAT top tracks (T1, T2) and the bottom GND track from JP1, so the copper carries the full-throttle current without heating.
- **Pack protection now comes from the firmware:** it refuses to arm below 3.3 V, warns below 3.5 V, lands by itself after 2 s below 3.3 V in flight (angle mode: it caps the trigger to aim for a 0.5 m/s descent, and touches down at 0.8-1.2 m/s in the sim; hold modes: it lands on the height sensor at about 0.4 m/s), and cuts the motors after 1 s below 3.0 V. Don't fly a pack that has been left flat.
- **Brown-out check (once, with props on and held down):** the pack sags about 0.6 V at full throttle, and the ESP32's 3.3 V regulator may dip with it. Hold the drone down, run full throttle for 2 s, and watch the serial monitor: a reboot, or "last reset was a BROWN-OUT" at the next start, means the rail dipped. If it does, fit a lower-resistance pack (a 40C pack sags less) or keep below full throttle until you can fit a larger capacitor at C1.

## 4. First power-up (props OFF)
1. Flash the firmware (`lite/firmware/README.md`) and open the serial monitor at 115200 baud.
2. **IMU check.** With the drone flat, `acc` should read about `0 0 9.8`. Lift the nose: `pitch` should go **negative**. Roll it right side down: `roll` should go **positive**. If not, fix `kImuToBody` in `lite_fc.ino`.
3. **Motor order and direction.** Send each of these in turn, at 0.2 duty:

   | Command | Motor | Should spin |
   |---|---|---|
   | `spin 0 0.2` | front-left | clockwise |
   | `spin 1 0.2` | front-right | counter-clockwise |
   | `spin 2 0.2` | rear-right | clockwise |
   | `spin 3 0.2` | rear-left | counter-clockwise |

   Directions are as seen from above (neuroflight's yaw sign). A motor that spins the wrong way: swap its two wires. A wrong position: swap the plugs.
4. Pair the Xbox controller: hold its pair button until the logo flashes fast. The LED should change to a slow blink.
5. Arm with Menu, and check that pulling the trigger a little spins all four motors. Then press B.

## 5. First flight
- Fit the props: front-left and rear-right take clockwise props, the other two counter-clockwise. Then charge the battery (off the drone).
- Fly in a large room or outdoors with no wind.
- Squeeze the trigger smoothly to about 60 %. It should lift off and stay level.
- If it wobbles fast, the rate gains are too high for your motors. Lower `kp`/`kd` in `RatePidConfig`. The sim shows the stock gains hold with motors up to 2× slower than estimated.
- If it slowly leans in a direction, that's the accelerometer-only attitude estimate (expected). Correct it with the stick.

## 6. v3 autonomy kit (optional): position hold and AprilTag vision
This adds three small boards. The drone then holds its position by itself, and a laptop can fly it relative to a printed AprilTag: follow it, approach and land in front of it, or turn to search for it. The flight board itself does not change.

| Part | What it does | Approx. cost |
|---|---|---|
| PMW3901 optical-flow breakout (19 × 14 mm class) | sees the floor move: horizontal speed | $10-15 |
| VL53L1X time-of-flight breakout (18 × 13 mm class) | height above the floor | $5-8 |
| Seeed XIAO ESP32-S3 **Sense** (with the camera board) | reads both sensors, streams the camera over Wi-Fi, relays laptop commands | $14-16 |
| 30 AWG silicone wire, 2 zip ties | | |

The kit boards add about $29-56, so the drone costs roughly $69-144 with the kit, depending on where you buy (see the BOM sheet). With the kit fitted the drone weighs about 77 g and flies about 6 min (the camera's Wi-Fi costs about 0.6 min), and the motors must reach 31 g each (step 1). Leave the prop guard off unless the motors reach 34 g.

The XIAO ESP32-S3 Sense needs its small external antenna plugged into the U.FL socket for the Wi-Fi video link; without it the range drops to a few metres. Tuck the antenna flat along the arch, clear of the props.

The height sensor runs in short mode at 50 Hz, which reads to about 1.3 m indoors. Above that the drone has no height and a hold mode lands, so keep autonomous flights below about 1.2 m.

**Mounting** (the v3 `frame.stl` has the mounts built in):
- Flow and ToF boards go on the tail tab, **sensor side down**, over the two windows. The flow board goes on the window on the drone's right (-y), the ToF board on its left. Fix them with a dot of hot glue or double-sided foam at the corners. Keep glue off the lens and the ToF aperture.
- The XIAO stands upright behind the nose arch, with the camera looking forward through the arch opening and USB-C at the top. Tie it with the two zip ties through the arch slots.

**Wiring** (keep the leads short and run them along the arms):

| From | To | Notes |
|---|---|---|
| VBAT / GND at the C1 capacitor pads (top side) | XIAO **BAT+ / BAT-** pads (underside) | powers the XIAO from the 1S pack. Unplug the battery before plugging USB into the XIAO, or its charger will try to charge the pack through the board. |
| XIAO 3V3 / GND | PMW3901 VCC / GND, VL53L1X VIN / GND | |
| XIAO D8 / D9 / D10 (SCK / MISO / MOSI), D1 | PMW3901 SCK / MISO / MOSI, CS | |
| XIAO D4 / D5 (SDA / SCL) | VL53L1X SDA / SCL | |
| XIAO D6 (TX) | ESP32 MiniKit **IO27** | companion data to the flight board |
| XIAO D7 (RX) | ESP32 MiniKit **IO25** | telemetry back to the XIAO |

IO25 and IO27 are free outer-row pins on the MiniKit. If your MiniKit's silkscreen differs, use any of 25, 27, 32 or 33 and change `kCompRxPin` / `kCompTxPin` in `lite_fc.ino`.

**Firmware:** flash `lite/firmware/companion/companion.ino` to the XIAO (see `lite/firmware/README.md`), then re-flash `lite_fc.ino`. Without the XIAO, the flight board behaves exactly like v2.

**Flow calibration (slide test, props off).** Open the XIAO's serial monitor at 115200 baud.
1. Hold the drone level about 30 cm above a textured floor (a rug or wood grain, not plain tiles). `range` should read about 300 mm.
2. Type `r`, then slide the drone 1.00 m **forward** along a tape measure, without tilting it. `slide x` should read about +1.0 and `slide y` about 0.
3. Type `r` and slide it 1.00 m to the **left**. `slide y` should read about +1.0.
4. If a sign is wrong or x/y are swapped, change `sign_x`, `sign_y` or `swap_xy` in `lite/companion.hpp` (`FlowConfig`). If the distance is off by a constant factor, scale `rad_per_count`. Then re-flash the XIAO.

**First autonomous flight:**
1. Arm and take off in angle mode as before, to about 1 m.
2. Press **A**: position hold. Release the trigger. The drone should hold its spot within about 10-15 cm. The right stick now moves it (up to 1 m/s) and the left stick y climbs or descends. **X** returns to angle mode (only with the trigger pulled, so it can't drop).
3. If the flow or height sensor drops out, the drone switches to height-only hold or lands. If you lose the controller in a hold mode, it lands. **B kills the motors in every mode.**
4. For vision: tape a printed tag (`python -m lite.ground.make_tags 0`) on a wall at about 1 m height. Start the ground station (`lite/ground/README.md`), hover in position hold facing the tag, press **Y** (AUTO), then press **G** in the station window.

## Checking the 3D model

`make lite-audit` checks the detailed model (`lite/detail.py`, 222 solid parts) on the exact CAD solids: every
part against every other part for overlaps, that every part is held by something that touches the frame, the
swept disc of each prop against everything else, that every part is one valid solid, and that no part is still
a plain placeholder box. It writes `lite/out/audit.json` and fails on any overlap, floating part or prop
clearance under 3 mm (v4; it was 1 mm, and the camera stack sat 1.6 mm under the front props).

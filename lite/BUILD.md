# Building the NF-3D Lite

The parts are in the "Micro Drone Parts List" doc. The files are in `lite/out/cad` (frame) and `lite/ecad/fab` (board).

## 1. Test the motors first (before you print or mill anything)
Brushed 8520 motors vary from batch to batch. The design flies on the model's nominal motors (about 29 g of thrust each at full power). It still flies on the pessimistic corner (about 22 g), but sluggishly. The flight sim shows it stays controllable, though there is less margin.
1. Wire one motor and its prop to a charged 1S battery through the board, or straight to the battery for 2 s.
2. Tape the motor upside down on a kitchen scale, so the prop blows upward and pushes down on the scale. Read the grams at full power.
3. **Each motor should show 25 g or more, and all four within about 2 g of each other.** The stress sim (`python -m lite.stress`) shows why matching matters. On weak motors, one motor 15 % weaker than the rest drops recovery from a throw to about 92 % (v2; v1 was about 60 %), and adding 15 % weight pushes hover to 88 % throttle, which leaves almost nothing for control. On normal motors both faults are harmless. At 22 g or less, the drone will struggle. Return those motors or buy a different batch. 1020 motors fit if you widen the motor tube (`motor_bore` in `lite/spec.yaml`).
4. Once the board is built, you can do the same test with the board's serial command `spin <motor> <duty>` (see below).

## 2. Print the frame
- `frame.stl`: **PETG** (v2), 0.2 mm layers, 3 walls, 100 % infill on the arms (30 % elsewhere is fine). Print it flat, as exported. It needs no supports. PETG is used because the motor cans get warm enough to soften PLA (PLA softens at about 55-60 °C, PETG at about 80 °C). PLA still works for a first test print.
- The print is about 11.3 g. If it comes out over 14 g, reduce the infill in the centre plate.
- **Motor clamps.** Each motor tube has a slit on its outboard side and a groove round it. Push the motor in until it sits on the ledge inside the tube, with its leads through the hole in the bottom, then pull a 2.5 mm zip tie tight in the groove. Print a test tube first if your printer runs tight or loose: `motor_bore` in `lite/spec.yaml` is the knob.
- **Battery.** It slides in from the front and stops against the two posts at the back. A strap goes through the two slots in the plate.
- **Optional prop guard** (`guard.stl`, 5.2 g, PETG, print flat): it drops over the tops of the four motor tubes. With it fitted the drone weighs about 72 g, hovers at about 60 % throttle and flies about 7.3 min (checked by `python -m lite.checks`). Fit it for your first indoor flights.

## 3. Make the board
Follow `lite/ecad/MILLING.md`. In short:
1. Mill the bottom side and drill the holes.
2. Make the 5 wire vias and the 7 top tracks.
3. Solder the parts, low ones first.
4. Trim all through-hole leads flush, because the battery sits under the board.

Two of the mounting screws (H3 and H4) must be **low button-head M2** screws.

**Two power-path changes before flying:**
- **Bridge JP1.** The charger module's protection trips at about 3 A, but hover alone draws about 3.7 A. Bridging JP1 bypasses it, so use a battery pack that has its own protection circuit (most 1S packs do). The firmware refuses to arm below 3.3 V and warns below 3.5 V.
- **Upgrade or bridge the slide switch.** The SS-12D00 is rated about 0.5 A. Either fit a higher-current switch on the same pads, or bridge COM to B and use the battery plug as the on/off switch.

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
- Fit the props: front-left and rear-right take clockwise props, the other two counter-clockwise. Then charge the battery.
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

This puts the build at roughly $60-75, above the original $50 target. With the kit fitted the drone weighs 77.9 g and flies about 6 min (the camera's Wi-Fi costs about 0.6 min). On pessimistic motors, leave the prop guard off: with both fitted the thrust-to-weight is only 1.18.

**Mounting** (the v3 `frame.stl` has the mounts built in):
- Flow and ToF boards go on the tail tab, **sensor side down**, over the two windows. The flow board goes on the window on the drone's right (-y), the ToF board on its left. Fix them with a dot of hot glue or double-sided foam at the corners. Keep glue off the lens and the ToF aperture.
- The XIAO stands upright behind the nose arch, with the camera looking forward through the arch opening and USB-C at the top. Tie it with the two zip ties through the arch slots.

**Wiring** (keep the leads short and run them along the arms):

| From | To | Notes |
|---|---|---|
| VBAT / GND at the C1 capacitor pads (top side) | XIAO **BAT+ / BAT-** pads (underside) | powers the XIAO from the 1S pack. Don't charge through the XIAO's USB and the TP4056 at the same time. |
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

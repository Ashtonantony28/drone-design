# Building the NF-3D Lite

The parts are in the "Micro Drone Parts List" doc. The files are in `lite/out/cad` (frame) and `lite/ecad/fab` (board).

## 1. Test the motors first (before you print or mill anything)
Brushed 8520 motors vary from batch to batch. The design flies on the model's nominal motors (about 29 g of thrust each at full power). It still flies on the pessimistic corner (about 22 g), but sluggishly. The flight sim shows it stays controllable, though there is less margin.
1. Wire one motor and its prop to a charged 1S battery through the board, or straight to the battery for 2 s.
2. Tape the motor upside down on a kitchen scale, so the prop blows upward and pushes down on the scale. Read the grams at full power.
3. **Each motor should show 25 g or more, and all four within about 2 g of each other.** The stress sim (`python -m lite.stress`) shows why matching matters. On weak motors, one motor 15 % weaker than the rest drops recovery from tilts to about 60 %. On normal motors the same fault is harmless. At 22 g or less, the drone will struggle. Return those motors or buy a different batch. 1020 motors fit if you widen the motor tube (`motor_bore` in `lite/spec.yaml`).
4. Once the board is built, you can do the same test with the board's serial command `spin <motor> <duty>` (see below).

## 2. Print the frame
- `frame.stl`: PLA, 0.2 mm layers, 3 walls, 30 % infill. Print it flat, as exported. It needs no supports.
- The print is about 15 g. If it comes out over 18 g, reduce the infill.
- The motor tubes are a press fit (8.5 mm bore). If they're loose, add one wrap of tape. If they're tight, ream with a drill bit.

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

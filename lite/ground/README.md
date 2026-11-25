# NF-3D Lite ground station (laptop)

The laptop watches the drone's camera, finds AprilTags (36h11 family) and steers the drone relative to them. The flight board does the flying: in AUTO it treats the laptop's messages as velocity commands for its own position hold. If the laptop goes quiet, the drone stops within 0.5 s and returns to position hold after 2 s.

## Setup (once)
```
pip install opencv-python numpy pyyaml
python -m lite.ground.make_tags 0 1 2      # lite/out/tags/tag_0.png ..., print at 100 % on A4
```
Check that the printed black square measures **150 mm**. Tape the tag flat on a wall at about 1 m height, in good light.

Optional, for better distance accuracy: print a 9 × 6 chessboard, then run `python -m lite.ground.calibrate --square 0.025` (your square size in metres). Pass the resulting `lite/ground/camera.yaml` to the station with `--camera lite/ground/camera.yaml`.

## Flying
1. Power the drone and join the Wi-Fi network **NF3D-Lite** (password `nf3d-lite`) on the laptop.
2. Run `python -m lite.ground.station --task follow` (or `--task land`). The video window opens **paused**, and the drone stays where it is.
3. With the Xbox controller: arm, take off, press **A** (position hold), point the drone at the tag, then press **Y** (AUTO).
4. Press **G** in the video window. The drone flies to 0.8 m in front of the tag and keeps facing it (`--dist` changes the distance). If you walk the tag around, the drone follows. With `--task land`, it lands 0.5 m in front of the tag.
5. **Space** pauses (the drone holds). **L** lands. **Q** quits. On the controller, **X** takes manual control back and **B** kills the motors.

If the tag is lost, the drone stops and holds. After 3 s in follow mode it turns slowly to look for the tag, towards the side where it was last seen.

Options: `--tag ID` (follow only this tag), `--no-window` (status in the terminal), `--record flight.mp4`.

## What has been tested (in the cloud, no hardware)
- `python -m pytest lite/ground/tests` (10 tests):
  - protocol frames byte-identical to the C++
  - parser fuzzing
  - tag pose accuracy on synthetic frames: error under 3 % of the range out to 3.5 m
  - follow, lost and search logic, and the landing trigger
  - a full run of this station app against a fake drone (MJPEG over HTTP, UDP)
- `python -m lite.vision_sim`: the whole loop in the flight simulator, with randomized vehicles and synthetic camera frames:
  - frames are JPEG-compressed, 15 fps, with 150 ms latency
  - this detector and controller produce the commands
  - the firmware's position hold flies them

  Scenarios: follow, a moving tag, approach-and-land, tag removed, and search. Results are in `lite/out/vision.json`.

Real-world gaps that the simulation can't close:
- motion blur and rolling shutter
- lighting changes
- Wi-Fi drop-outs, beyond the stale-command handling
- the real lens's field of view (calibrate it)

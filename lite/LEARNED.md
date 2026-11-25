# Learned controller on the Lite (neuroflight-fc hover_ctbr_v3)

Question: can the Lite fly neuroflight-fc's sim-trained neural controller, and should it run on the
drone or on the laptop? Short answer: **yes, onboard on the flight board's ESP32, with the thrust
rescaled to the Lite; never over Wi-Fi.** The laptop keeps doing what it does now (vision, slow
velocity commands).

## Does it fit and run fast enough? (`lite/firmware/policy_bench`)
- The policy is a 19-64-64-4 MLP (5,568 multiply-adds per step), run at 500 Hz ahead of the 1 kHz rate PID.
- Both the float32 and int8 models together take **32 kB of flash** (compiled for the ESP32 with
  arduino-cli); RAM use is two 128-float scratch buffers on the stack. The flight board has 540 kB of flash left.
- Time per step, estimated from the compiled inner loop (about 9 instructions per multiply-add) plus
  128 `tanhf` calls: roughly **0.3-0.6 ms float32, 0.15-0.4 ms int8**, against a 2 ms budget at 500 Hz.
  It fits, but it is the biggest job on core 1. Flash `policy_bench` to measure it on the real board.
- It needs the v3 autonomy kit: the policy reads position and velocity from the flow/ToF filter.

## Does the existing policy fly the Lite? (`python -m lite.learned`, n = 128, onboard 3 ms delay)
| controller | hover_1m nom / rand | tilt_60 nom / rand | throw nom / rand |
|---|---|---|---|
| v3 policy as is | 0 / 0 % | 0 / 0 % | 0 / 0 % |
| v3 policy, thrust rescaled to the Lite | 100 / 100 % | 100 / 98.4 % | 100 / 100 % |
| neuroflight PID on the Lite | 100 / 97.7 % | 99.2 / 99.2 % | 97.7 / 94.5 % |
| Lite's own position hold | 100 / 100 % | 100 / 100 % | 99.2 / 93.0 % |

As is, it fails because it was trained on a 250 g drone with thrust/weight 7.3 (hover at 14 %
throttle); the Lite is 78 g with thrust/weight 1.8 (hover at 60 %). Rescaling the collective thrust
by the mass/thrust ratio (`flight/fly.py CollectiveScaled`, the same fix the v1 drone uses) is enough
in simulation. A retrain on the Lite's own model would be the proper next step before relying on it.

## Onboard or on the laptop? (whole control loop delayed by the Wi-Fi round trip)
Success on the randomized vehicles (worst of hover_1m / tilt_60 / throw):

| delay | v3 policy (own vehicle) | PID on the Lite | Lite position hold |
|---|---|---|---|
| 3 ms (onboard) | 87.5 % | 94.5 % | 93.0 % |
| 10 ms (best-case Wi-Fi) | 76.6 % | 75.0 % | 91.4 % |
| 25 ms (typical) | 6.2 % | 3.1 % | 25.0 % |
| 50 ms, 100 ms | 0 % | 0 % | 0 % |

A 2.4 GHz link to a laptop is 10-50 ms with spikes past 100 ms, so any stabilizing loop on the
laptop crashes the drone. Real Wi-Fi drop-outs (not modelled) make it worse. The split the Lite already uses is the right one:
attitude, rates, position hold (or the learned policy) on the drone; vision and slow velocity
commands on the laptop, with the drone stopping on its own if the laptop goes quiet.

# Flight validation (neuroflight-fc simulator, firmware sensing path `fw3`)

Controllers: `PID` = neuroflight-fc cascaded fallback (always on board), `v3` = hover_ctbr_v3 as trained on the reference vehicle, `v3cal` = the same v3 weights with the collective-thrust output scaled to this airframe's mass and thrust (flight/fly.py CollectiveScaled, no retraining).

**Airframe gate:** PID success on hover_1m/randomized, tilt_60/randomized, throw/randomized at most 3 points below the same controller on the reference vehicle (neuroflight-fc config/vehicle_nominal.yaml), for both the nominal and the pessimistic NF-3D. **Learned policies:** scored the same way, reported as findings.

## reference vehicle

```
mass: 0.25
inertia:
- 0.00025
- 0.00025
- 0.0004
arm_xy: 0.046
thrust_max: 4.5
thrust_curve_a: 0.7
yaw_torque_coeff: 0.006
```

| scenario | succ% v3 | succ% PID | succ% v3cal | v3 err_med m | PID err_med m | v3cal err_med m | v3 worst/p95 tilt° | PID worst/p95 tilt° | v3cal worst/p95 tilt° | v3 settle s | PID settle s | v3cal settle s | v3 trip% | PID trip% | v3cal trip% | v3 sup% | PID sup% | v3cal sup% | v3 HF rate/thr | PID HF rate/thr | v3cal HF rate/thr | v3 thr/PID | v3cal thr/PID |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover_1m/nominal | 100.0 | 99.6 | 100.0 | 0.035 | 0.019 | 0.035 | 36/33 | 38/36 | 36/33 | 0.94 | 0.91 | 0.94 | 0.0 | 0.0 | 0.0 | 0.0 | 0.8 | 0.0 | 0.0014/0.0011 | 0.0008/0.0003 | 0.0014/0.0011 | 4.32 | 4.32 |
| hover_1m/randomized | 100.0 | 98.4 | 100.0 | 0.033 | 0.027 | 0.033 | 37/33 | 40/37 | 37/33 | 0.95 | 0.96 | 0.95 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0019/0.0017 | 0.0027/0.0013 | 0.0019/0.0017 | 1.31 | 1.30 |
| tilt_60/nominal | 91.0 | 99.6 | 91.4 | 0.058 | 0.021 | 0.058 | 60/60 | 60/60 | 60/60 | 0.76 | 0.77 | 0.76 | 0.0 | 0.0 | 0.0 | 3.1 | 0.4 | 3.1 | 0.0013/0.0010 | 0.0008/0.0001 | 0.0013/0.0010 | 8.81 | 8.80 |
| tilt_60/randomized | 90.6 | 96.1 | 90.6 | 0.059 | 0.031 | 0.059 | 60/60 | 60/60 | 60/60 | 0.83 | 0.84 | 0.83 | 0.0 | 0.0 | 0.0 | 2.3 | 0.0 | 2.3 | 0.0019/0.0018 | 0.0029/0.0014 | 0.0019/0.0018 | 1.36 | 1.36 |
| throw/nominal | 98.0 | 99.6 | 98.0 | 0.039 | 0.026 | 0.039 | 46/40 | 45/42 | 46/40 | 0.70 | 0.63 | 0.70 | 1.6 | 2.0 | 1.6 | 1.6 | 2.3 | 1.6 | 0.0015/0.0012 | 0.0009/0.0002 | 0.0015/0.0012 | 7.94 | 7.96 |
| throw/randomized | 97.3 | 95.3 | 97.3 | 0.039 | 0.032 | 0.039 | 44/40 | 62/42 | 44/40 | 0.61 | 0.66 | 0.61 | 2.3 | 3.9 | 2.3 | 2.3 | 4.7 | 2.3 | 0.0020/0.0018 | 0.0031/0.0013 | 0.0020/0.0019 | 1.37 | 1.37 |
| upside_down/nominal | 68.0 | 75.0 | 68.0 | 0.081 | 0.069 | 0.081 | 180/179 | 180/178 | 180/179 | 1.27 | 1.28 | 1.27 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0017/0.0015 | 0.0012/0.0006 | 0.0017/0.0015 | 2.67 | 2.67 |
| upside_down/randomized | 61.3 | 68.0 | 61.3 | 0.090 | 0.075 | 0.090 | 180/179 | 180/179 | 180/179 | 1.35 | 1.56 | 1.35 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0021/0.0020 | 0.0034/0.0015 | 0.0021/0.0020 | 1.36 | 1.35 |

Randomization extremes (one parameter at min/max, rest nominal, no other noise): success %

| param=extreme | v3 tilt_60 | PID tilt_60 | v3cal tilt_60 | v3 throw | PID throw | v3cal throw |
|---|---|---|---|---|---|---|
| mass=min | 94 | 100 | 94 | 100 | 100 | 100 |
| mass=max | 91 | 94 | 91 | 97 | 100 | 97 |
| inertia=min | 88 | 100 | 88 | 100 | 97 | 100 |
| inertia=max | 100 | 100 | 100 | 97 | 100 | 97 |
| thrust_max=min | 91 | 97 | 91 | 100 | 97 | 100 |
| thrust_max=max | 88 | 100 | 88 | 100 | 94 | 100 |
| per_motor_thrust_imbalance=min | 88 | 100 | 88 | 100 | 100 | 100 |
| per_motor_thrust_imbalance=max | 94 | 100 | 94 | 100 | 94 | 100 |
| yaw_torque_coeff=min | 94 | 100 | 94 | 97 | 97 | 97 |
| yaw_torque_coeff=max | 100 | 97 | 100 | 100 | 100 | 100 |
| motor_tau=min | 94 | 100 | 94 | 100 | 100 | 100 |
| motor_tau=max | 81 | 100 | 81 | 100 | 100 | 100 |
| lin_drag=min | 94 | 100 | 94 | 97 | 97 | 97 |
| lin_drag=max | 84 | 100 | 84 | 100 | 97 | 100 |
| latency_s=min | 100 | 97 | 100 | 100 | 100 | 100 |
| latency_s=max | 91 | 97 | 91 | 100 | 100 | 100 |
| gyro_noise_std=min | 97 | 100 | 97 | 100 | 97 | 100 |
| gyro_noise_std=max | 94 | 100 | 91 | 97 | 97 | 97 |
| gyro_bias=min | 97 | 97 | 97 | 100 | 94 | 100 |
| gyro_bias=max | 97 | 100 | 97 | 100 | 100 | 100 |
| accel_noise_std=min | 91 | 100 | 91 | 97 | 97 | 97 |
| accel_noise_std=max | 94 | 100 | 94 | 100 | 100 | 100 |
| vel_noise_std=min | 94 | 100 | 94 | 100 | 100 | 100 |
| vel_noise_std=max | 84 | 97 | 84 | 100 | 100 | 100 |
| pos_noise_std=min | 100 | 97 | 100 | 100 | 100 | 100 |
| pos_noise_std=max | 97 | 100 | 97 | 97 | 100 | 97 |

- v3: worst extreme `motor_tau=max/tilt_60` -> 81%
- PID: worst extreme `mass=max/tilt_60` -> 94%
- v3cal: worst extreme `motor_tau=max/tilt_60` -> 81%

## nominal vehicle

```
mass: 0.232
inertia:
- 0.0004047
- 0.0004168
- 0.000703
arm_xy: 0.05
thrust_max: 3.343
thrust_curve_a: 0.2656
yaw_torque_coeff: 0.00758
```

| scenario | succ% v3 | succ% PID | succ% v3cal | v3 err_med m | PID err_med m | v3cal err_med m | v3 worst/p95 tilt° | PID worst/p95 tilt° | v3cal worst/p95 tilt° | v3 settle s | PID settle s | v3cal settle s | v3 trip% | PID trip% | v3cal trip% | v3 sup% | PID sup% | v3cal sup% | v3 HF rate/thr | PID HF rate/thr | v3cal HF rate/thr | v3 thr/PID | v3cal thr/PID |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover_1m/nominal | 100.0 | 100.0 | 100.0 | 0.027 | 0.019 | 0.027 | 37/33 | 38/37 | 36/34 | 0.95 | 0.91 | 0.94 | 0.0 | 0.0 | 0.0 | 0.0 | 1.6 | 0.0 | 0.0016/0.0011 | 0.0009/0.0003 | 0.0015/0.0013 | 3.57 | 4.42 |
| hover_1m/randomized | 99.2 | 98.8 | 100.0 | 0.043 | 0.027 | 0.034 | 37/33 | 42/38 | 38/33 | 0.97 | 0.95 | 0.95 | 0.0 | 0.0 | 0.0 | 0.0 | 3.1 | 0.0 | 0.0021/0.0017 | 0.0028/0.0016 | 0.0020/0.0020 | 1.05 | 1.27 |
| tilt_60/nominal | 94.5 | 99.6 | 89.8 | 0.054 | 0.032 | 0.056 | 60/60 | 60/60 | 60/60 | 0.81 | 0.80 | 0.82 | 0.0 | 0.0 | 0.0 | 15.6 | 5.9 | 10.2 | 0.0016/0.0011 | 0.0008/0.0001 | 0.0014/0.0014 | 8.34 | 10.76 |
| tilt_60/randomized | 85.9 | 97.3 | 89.5 | 0.065 | 0.039 | 0.059 | 60/60 | 60/60 | 60/60 | 0.86 | 0.85 | 0.88 | 0.0 | 0.0 | 0.0 | 10.9 | 9.0 | 5.5 | 0.0020/0.0017 | 0.0026/0.0017 | 0.0019/0.0022 | 1.02 | 1.29 |
| throw/nominal | 98.8 | 97.3 | 98.0 | 0.035 | 0.027 | 0.033 | 46/41 | 48/43 | 47/40 | 0.73 | 0.64 | 0.72 | 1.6 | 2.0 | 1.6 | 2.0 | 4.3 | 1.6 | 0.0018/0.0012 | 0.0015/0.0003 | 0.0016/0.0015 | 4.48 | 5.66 |
| throw/randomized | 95.3 | 95.7 | 98.0 | 0.047 | 0.034 | 0.037 | 45/41 | 65/44 | 48/41 | 0.63 | 0.61 | 0.64 | 2.7 | 4.3 | 2.7 | 4.3 | 7.0 | 4.7 | 0.0021/0.0019 | 0.0032/0.0017 | 0.0021/0.0023 | 1.11 | 1.32 |
| upside_down/nominal | 61.7 | 81.2 | 63.7 | 0.087 | 0.061 | 0.085 | 180/179 | 180/178 | 180/179 | 1.39 | 1.43 | 1.31 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0018/0.0020 | 0.0013/0.0007 | 0.0017/0.0019 | 2.61 | 2.56 |
| upside_down/randomized | 51.6 | 73.4 | 56.6 | 0.097 | 0.072 | 0.093 | 180/179 | 180/179 | 180/179 | 1.49 | 1.64 | 1.44 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0022/0.0024 | 0.0030/0.0018 | 0.0021/0.0025 | 1.30 | 1.39 |

Randomization extremes (one parameter at min/max, rest nominal, no other noise): success %

| param=extreme | v3 tilt_60 | PID tilt_60 | v3cal tilt_60 | v3 throw | PID throw | v3cal throw |
|---|---|---|---|---|---|---|
| mass=min | 94 | 100 | 84 | 100 | 100 | 100 |
| mass=max | 91 | 97 | 100 | 100 | 100 | 100 |
| inertia=min | 91 | 100 | 97 | 94 | 97 | 97 |
| inertia=max | 97 | 100 | 91 | 97 | 94 | 94 |
| thrust_max=min | 94 | 100 | 91 | 97 | 100 | 100 |
| thrust_max=max | 94 | 97 | 91 | 100 | 94 | 100 |
| per_motor_thrust_imbalance=min | 91 | 100 | 91 | 100 | 91 | 100 |
| per_motor_thrust_imbalance=max | 94 | 97 | 94 | 100 | 94 | 100 |
| yaw_torque_coeff=min | 94 | 100 | 94 | 100 | 94 | 100 |
| yaw_torque_coeff=max | 94 | 100 | 97 | 97 | 97 | 100 |
| motor_tau=min | 97 | 97 | 97 | 100 | 97 | 100 |
| motor_tau=max | 94 | 100 | 84 | 100 | 88 | 100 |
| lin_drag=min | 94 | 94 | 84 | 100 | 100 | 100 |
| lin_drag=max | 91 | 100 | 91 | 100 | 100 | 100 |
| latency_s=min | 94 | 100 | 100 | 100 | 100 | 100 |
| latency_s=max | 78 | 100 | 91 | 100 | 97 | 100 |
| gyro_noise_std=min | 94 | 100 | 94 | 94 | 91 | 100 |
| gyro_noise_std=max | 91 | 100 | 97 | 100 | 97 | 100 |
| gyro_bias=min | 97 | 100 | 81 | 94 | 94 | 100 |
| gyro_bias=max | 88 | 100 | 91 | 100 | 100 | 94 |
| accel_noise_std=min | 97 | 100 | 91 | 100 | 100 | 97 |
| accel_noise_std=max | 97 | 100 | 94 | 100 | 94 | 97 |
| vel_noise_std=min | 97 | 100 | 94 | 100 | 100 | 97 |
| vel_noise_std=max | 88 | 100 | 91 | 100 | 97 | 100 |
| pos_noise_std=min | 97 | 100 | 94 | 100 | 94 | 100 |
| pos_noise_std=max | 91 | 97 | 94 | 100 | 100 | 100 |

- v3: worst extreme `latency_s=max/tilt_60` -> 78%
- PID: worst extreme `motor_tau=max/throw` -> 88%
- v3cal: worst extreme `gyro_bias=min/tilt_60` -> 81%

## pessimistic vehicle

```
mass: 0.2436
inertia:
- 0.0004249
- 0.0004376
- 0.0007382
arm_xy: 0.05
thrust_max: 2.56
thrust_curve_a: 0.1966
yaw_torque_coeff: 0.0107
```

| scenario | succ% v3 | succ% PID | succ% v3cal | v3 err_med m | PID err_med m | v3cal err_med m | v3 worst/p95 tilt° | PID worst/p95 tilt° | v3cal worst/p95 tilt° | v3 settle s | PID settle s | v3cal settle s | v3 trip% | PID trip% | v3cal trip% | v3 sup% | PID sup% | v3cal sup% | v3 HF rate/thr | PID HF rate/thr | v3cal HF rate/thr | v3 thr/PID | v3cal thr/PID |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover_1m/nominal | 94.5 | 99.2 | 100.0 | 0.088 | 0.019 | 0.035 | 37/33 | 39/37 | 36/33 | 1.11 | 0.91 | 0.93 | 0.0 | 0.0 | 0.0 | 0.0 | 2.7 | 0.0 | 0.0015/0.0012 | 0.0009/0.0004 | 0.0014/0.0019 | 2.73 | 4.35 |
| hover_1m/randomized | 50.0 | 98.0 | 100.0 | 0.099 | 0.029 | 0.035 | 36/33 | 42/38 | 37/33 | 1.03 | 0.95 | 0.94 | 0.0 | 0.0 | 0.0 | 0.0 | 5.5 | 0.0 | 0.0021/0.0017 | 0.0028/0.0022 | 0.0019/0.0028 | 0.79 | 1.30 |
| tilt_60/nominal | 49.2 | 98.0 | 85.9 | 0.100 | 0.028 | 0.059 | 60/60 | 60/60 | 60/60 | 0.93 | 0.79 | 0.82 | 0.0 | 0.0 | 0.0 | 13.3 | 2.7 | 3.9 | 0.0015/0.0004 | 0.0008/0.0002 | 0.0014/0.0018 | 2.59 | 10.76 |
| tilt_60/randomized | 33.6 | 96.5 | 90.2 | 0.115 | 0.035 | 0.059 | 60/60 | 60/60 | 60/60 | 0.89 | 0.86 | 0.87 | 0.0 | 0.0 | 0.0 | 8.6 | 0.4 | 5.5 | 0.0020/0.0013 | 0.0028/0.0023 | 0.0019/0.0030 | 0.59 | 1.28 |
| throw/nominal | 75.0 | 98.4 | 97.7 | 0.091 | 0.028 | 0.040 | 47/40 | 48/43 | 47/41 | 0.81 | 0.65 | 0.72 | 1.6 | 2.0 | 1.6 | 3.1 | 4.3 | 2.0 | 0.0017/0.0007 | 0.0012/0.0003 | 0.0015/0.0020 | 2.04 | 5.73 |
| throw/randomized | 54.3 | 96.9 | 98.0 | 0.097 | 0.032 | 0.040 | 52/41 | 61/44 | 46/41 | 0.67 | 0.62 | 0.64 | 2.3 | 4.3 | 2.3 | 3.1 | 5.5 | 2.3 | 0.0022/0.0016 | 0.0032/0.0023 | 0.0020/0.0031 | 0.67 | 1.34 |
| upside_down/nominal | 22.7 | 74.6 | 71.9 | 0.122 | 0.071 | 0.078 | 180/179 | 180/178 | 180/179 | 1.82 | 1.44 | 1.32 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0018/0.0019 | 0.0013/0.0010 | 0.0016/0.0026 | 1.87 | 2.55 |
| upside_down/randomized | 22.3 | 73.0 | 62.9 | 0.138 | 0.076 | 0.085 | 180/179 | 180/179 | 180/179 | 1.67 | 1.64 | 1.40 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0023/0.0024 | 0.0033/0.0025 | 0.0021/0.0035 | 0.98 | 1.40 |

Randomization extremes (one parameter at min/max, rest nominal, no other noise): success %

| param=extreme | v3 tilt_60 | PID tilt_60 | v3cal tilt_60 | v3 throw | PID throw | v3cal throw |
|---|---|---|---|---|---|---|
| mass=min | 81 | 97 | 97 | 100 | 100 | 100 |
| mass=max | 0 | 100 | 94 | 0 | 100 | 100 |
| inertia=min | 47 | 97 | 81 | 81 | 94 | 97 |
| inertia=max | 41 | 100 | 94 | 81 | 97 | 97 |
| thrust_max=min | 0 | 97 | 100 | 0 | 97 | 100 |
| thrust_max=max | 84 | 94 | 91 | 97 | 97 | 100 |
| per_motor_thrust_imbalance=min | 34 | 91 | 75 | 81 | 94 | 100 |
| per_motor_thrust_imbalance=max | 41 | 97 | 91 | 59 | 91 | 97 |
| yaw_torque_coeff=min | 53 | 100 | 88 | 66 | 88 | 100 |
| yaw_torque_coeff=max | 47 | 97 | 94 | 72 | 88 | 97 |
| motor_tau=min | 66 | 100 | 97 | 78 | 100 | 100 |
| motor_tau=max | 34 | 94 | 91 | 72 | 88 | 100 |
| lin_drag=min | 44 | 97 | 88 | 84 | 100 | 100 |
| lin_drag=max | 56 | 100 | 84 | 78 | 97 | 100 |
| latency_s=min | 44 | 91 | 91 | 88 | 100 | 100 |
| latency_s=max | 53 | 100 | 81 | 56 | 94 | 100 |
| gyro_noise_std=min | 53 | 97 | 94 | 75 | 97 | 97 |
| gyro_noise_std=max | 53 | 100 | 88 | 78 | 97 | 100 |
| gyro_bias=min | 56 | 100 | 81 | 72 | 88 | 100 |
| gyro_bias=max | 72 | 100 | 94 | 72 | 100 | 94 |
| accel_noise_std=min | 41 | 100 | 91 | 72 | 100 | 94 |
| accel_noise_std=max | 69 | 100 | 88 | 88 | 97 | 97 |
| vel_noise_std=min | 50 | 97 | 81 | 78 | 97 | 100 |
| vel_noise_std=max | 47 | 94 | 88 | 62 | 97 | 97 |
| pos_noise_std=min | 53 | 100 | 91 | 88 | 100 | 100 |
| pos_noise_std=max | 38 | 100 | 78 | 91 | 97 | 100 |

- v3: worst extreme `mass=max/tilt_60` -> 0%
- PID: worst extreme `yaw_torque_coeff=min/throw` -> 88%
- v3cal: worst extreme `per_motor_thrust_imbalance=min/tilt_60` -> 75%

## Airframe gate

PASS

## Learned-policy findings

- nominal: v3 tilt_60/randomized 85.9 % < reference v3 90.6 % - 3
- pessimistic: v3 hover_1m/randomized 50.0 % < reference v3 100.0 % - 3
- pessimistic: v3 tilt_60/randomized 33.6 % < reference v3 90.6 % - 3
- pessimistic: v3 throw/randomized 54.3 % < reference v3 97.3 % - 3

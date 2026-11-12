# Flight validation (neuroflight-fc simulator, firmware sensing path `fw3`)

Controllers: `PID` = neuroflight-fc cascaded fallback (always on board), `v3` = hover_ctbr_v3 as trained on the reference vehicle, `nf3d` = v3 fine-tuned on this airframe (flight/retrain.py).

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

| scenario | succ% v3 | succ% PID | succ% nf3d | v3 err_med m | PID err_med m | nf3d err_med m | v3 worst/p95 tilt° | PID worst/p95 tilt° | nf3d worst/p95 tilt° | v3 settle s | PID settle s | nf3d settle s | v3 trip% | PID trip% | nf3d trip% | v3 sup% | PID sup% | nf3d sup% | v3 HF rate/thr | PID HF rate/thr | nf3d HF rate/thr | v3 thr/PID | nf3d thr/PID |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover_1m/nominal | 100.0 | 99.6 | 100.0 | 0.035 | 0.019 | 0.037 | 36/33 | 38/36 | 32/28 | 0.94 | 0.91 | 1.06 | 0.0 | 0.0 | 0.0 | 0.0 | 0.8 | 0.0 | 0.0014/0.0011 | 0.0008/0.0003 | 0.0013/0.0010 | 4.32 | 3.86 |
| hover_1m/randomized | 100.0 | 98.4 | 100.0 | 0.033 | 0.027 | 0.044 | 37/33 | 40/37 | 34/30 | 0.95 | 0.96 | 1.10 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0019/0.0017 | 0.0027/0.0013 | 0.0016/0.0018 | 1.31 | 1.39 |
| tilt_60/nominal | 91.0 | 99.6 | 83.2 | 0.058 | 0.021 | 0.068 | 60/60 | 60/60 | 60/60 | 0.76 | 0.77 | 1.17 | 0.0 | 0.0 | 0.0 | 3.1 | 0.4 | 5.1 | 0.0013/0.0010 | 0.0008/0.0001 | 0.0015/0.0012 | 8.81 | 9.90 |
| tilt_60/randomized | 90.6 | 96.1 | 85.9 | 0.059 | 0.031 | 0.070 | 60/60 | 60/60 | 60/60 | 0.83 | 0.84 | 1.23 | 0.0 | 0.0 | 0.0 | 2.3 | 0.0 | 1.6 | 0.0019/0.0018 | 0.0029/0.0014 | 0.0017/0.0018 | 1.36 | 1.33 |
| throw/nominal | 98.0 | 99.6 | 98.8 | 0.039 | 0.026 | 0.041 | 46/40 | 45/42 | 46/40 | 0.70 | 0.63 | 0.95 | 1.6 | 2.0 | 1.6 | 1.6 | 2.3 | 1.6 | 0.0015/0.0012 | 0.0009/0.0002 | 0.0014/0.0013 | 7.94 | 8.58 |
| throw/randomized | 97.3 | 95.3 | 96.9 | 0.039 | 0.032 | 0.050 | 44/40 | 62/42 | 46/40 | 0.61 | 0.66 | 0.92 | 2.3 | 3.9 | 2.3 | 2.3 | 4.7 | 3.1 | 0.0020/0.0018 | 0.0031/0.0013 | 0.0018/0.0019 | 1.37 | 1.43 |
| upside_down/nominal | 68.0 | 75.0 | 52.7 | 0.081 | 0.069 | 0.092 | 180/179 | 180/178 | 180/179 | 1.27 | 1.28 | 1.65 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0017/0.0015 | 0.0012/0.0006 | 0.0017/0.0017 | 2.67 | 2.93 |
| upside_down/randomized | 61.3 | 68.0 | 51.2 | 0.090 | 0.075 | 0.095 | 180/179 | 180/179 | 180/179 | 1.35 | 1.56 | 1.76 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0021/0.0020 | 0.0034/0.0015 | 0.0020/0.0021 | 1.36 | 1.45 |

Randomization extremes (one parameter at min/max, rest nominal, no other noise): success %

| param=extreme | v3 tilt_60 | PID tilt_60 | nf3d tilt_60 | v3 throw | PID throw | nf3d throw |
|---|---|---|---|---|---|---|
| mass=min | 94 | 100 | 75 | 100 | 100 | 97 |
| mass=max | 91 | 94 | 91 | 97 | 100 | 100 |
| inertia=min | 88 | 100 | 81 | 100 | 97 | 97 |
| inertia=max | 100 | 100 | 84 | 97 | 100 | 97 |
| thrust_max=min | 91 | 97 | 94 | 100 | 97 | 100 |
| thrust_max=max | 88 | 100 | 88 | 100 | 94 | 100 |
| per_motor_thrust_imbalance=min | 88 | 100 | 88 | 100 | 100 | 100 |
| per_motor_thrust_imbalance=max | 94 | 100 | 88 | 100 | 94 | 100 |
| yaw_torque_coeff=min | 94 | 100 | 81 | 97 | 97 | 97 |
| yaw_torque_coeff=max | 100 | 97 | 88 | 100 | 100 | 100 |
| motor_tau=min | 94 | 100 | 78 | 100 | 100 | 100 |
| motor_tau=max | 81 | 100 | 84 | 100 | 100 | 100 |
| lin_drag=min | 94 | 100 | 88 | 97 | 97 | 100 |
| lin_drag=max | 84 | 100 | 78 | 100 | 97 | 100 |
| latency_s=min | 100 | 97 | 94 | 100 | 100 | 100 |
| latency_s=max | 91 | 97 | 81 | 100 | 100 | 100 |
| gyro_noise_std=min | 97 | 100 | 75 | 100 | 97 | 100 |
| gyro_noise_std=max | 94 | 100 | 97 | 97 | 97 | 100 |
| gyro_bias=min | 97 | 97 | 84 | 100 | 94 | 97 |
| gyro_bias=max | 97 | 100 | 72 | 100 | 100 | 97 |
| accel_noise_std=min | 91 | 100 | 88 | 97 | 97 | 100 |
| accel_noise_std=max | 94 | 100 | 94 | 100 | 100 | 100 |
| vel_noise_std=min | 94 | 100 | 84 | 100 | 100 | 100 |
| vel_noise_std=max | 84 | 97 | 81 | 100 | 100 | 100 |
| pos_noise_std=min | 100 | 97 | 78 | 100 | 100 | 100 |
| pos_noise_std=max | 97 | 100 | 88 | 97 | 100 | 100 |

- v3: worst extreme `motor_tau=max/tilt_60` -> 81%
- PID: worst extreme `mass=max/tilt_60` -> 94%
- nf3d: worst extreme `gyro_bias=max/tilt_60` -> 72%

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

| scenario | succ% v3 | succ% PID | succ% nf3d | v3 err_med m | PID err_med m | nf3d err_med m | v3 worst/p95 tilt° | PID worst/p95 tilt° | nf3d worst/p95 tilt° | v3 settle s | PID settle s | nf3d settle s | v3 trip% | PID trip% | nf3d trip% | v3 sup% | PID sup% | nf3d sup% | v3 HF rate/thr | PID HF rate/thr | nf3d HF rate/thr | v3 thr/PID | nf3d thr/PID |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover_1m/nominal | 100.0 | 100.0 | 100.0 | 0.027 | 0.019 | 0.054 | 37/33 | 38/37 | 33/29 | 0.95 | 0.91 | 1.03 | 0.0 | 0.0 | 0.0 | 0.0 | 1.6 | 0.0 | 0.0016/0.0011 | 0.0009/0.0003 | 0.0012/0.0009 | 3.57 | 2.81 |
| hover_1m/randomized | 99.2 | 98.8 | 96.9 | 0.043 | 0.027 | 0.058 | 37/33 | 42/38 | 34/30 | 0.97 | 0.95 | 1.06 | 0.0 | 0.0 | 0.0 | 0.0 | 3.1 | 0.0 | 0.0021/0.0017 | 0.0028/0.0016 | 0.0016/0.0017 | 1.05 | 1.05 |
| tilt_60/nominal | 94.5 | 99.6 | 87.1 | 0.054 | 0.032 | 0.073 | 60/60 | 60/60 | 60/60 | 0.81 | 0.80 | 1.10 | 0.0 | 0.0 | 0.0 | 15.6 | 5.9 | 7.8 | 0.0016/0.0011 | 0.0008/0.0001 | 0.0012/0.0010 | 8.34 | 7.46 |
| tilt_60/randomized | 85.9 | 97.3 | 77.0 | 0.065 | 0.039 | 0.075 | 60/60 | 60/60 | 60/60 | 0.86 | 0.85 | 1.13 | 0.0 | 0.0 | 0.0 | 10.9 | 9.0 | 6.6 | 0.0020/0.0017 | 0.0026/0.0017 | 0.0016/0.0018 | 1.02 | 1.07 |
| throw/nominal | 98.8 | 97.3 | 99.2 | 0.035 | 0.027 | 0.058 | 46/41 | 48/43 | 48/40 | 0.73 | 0.64 | 0.92 | 1.6 | 2.0 | 1.6 | 2.0 | 4.3 | 1.6 | 0.0018/0.0012 | 0.0015/0.0003 | 0.0014/0.0011 | 4.50 | 4.07 |
| throw/randomized | 95.3 | 95.7 | 93.4 | 0.047 | 0.034 | 0.062 | 45/41 | 65/44 | 46/39 | 0.63 | 0.61 | 0.84 | 2.7 | 4.3 | 2.3 | 4.3 | 7.0 | 5.1 | 0.0021/0.0019 | 0.0032/0.0017 | 0.0017/0.0018 | 1.11 | 1.05 |
| upside_down/nominal | 61.7 | 81.2 | 42.6 | 0.087 | 0.061 | 0.104 | 180/179 | 180/178 | 180/178 | 1.39 | 1.43 | 1.73 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0018/0.0020 | 0.0013/0.0007 | 0.0015/0.0019 | 2.61 | 2.49 |
| upside_down/randomized | 51.6 | 73.4 | 43.4 | 0.097 | 0.072 | 0.102 | 180/179 | 180/179 | 180/179 | 1.49 | 1.64 | 1.82 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0022/0.0024 | 0.0030/0.0018 | 0.0018/0.0022 | 1.30 | 1.23 |

Randomization extremes (one parameter at min/max, rest nominal, no other noise): success %

| param=extreme | v3 tilt_60 | PID tilt_60 | nf3d tilt_60 | v3 throw | PID throw | nf3d throw |
|---|---|---|---|---|---|---|
| mass=min | 94 | 100 | 94 | 100 | 100 | 100 |
| mass=max | 91 | 97 | 78 | 100 | 100 | 100 |
| inertia=min | 91 | 100 | 78 | 94 | 97 | 94 |
| inertia=max | 97 | 100 | 94 | 97 | 94 | 100 |
| thrust_max=min | 94 | 100 | 78 | 97 | 100 | 84 |
| thrust_max=max | 94 | 97 | 97 | 100 | 94 | 100 |
| per_motor_thrust_imbalance=min | 91 | 100 | 94 | 100 | 91 | 100 |
| per_motor_thrust_imbalance=max | 94 | 97 | 84 | 100 | 94 | 100 |
| yaw_torque_coeff=min | 94 | 100 | 91 | 100 | 94 | 97 |
| yaw_torque_coeff=max | 94 | 100 | 88 | 97 | 97 | 100 |
| motor_tau=min | 97 | 97 | 84 | 100 | 97 | 100 |
| motor_tau=max | 94 | 100 | 88 | 100 | 88 | 100 |
| lin_drag=min | 94 | 94 | 84 | 100 | 100 | 97 |
| lin_drag=max | 91 | 100 | 88 | 100 | 100 | 100 |
| latency_s=min | 94 | 100 | 88 | 100 | 100 | 100 |
| latency_s=max | 78 | 100 | 75 | 100 | 97 | 97 |
| gyro_noise_std=min | 94 | 100 | 91 | 94 | 91 | 94 |
| gyro_noise_std=max | 91 | 100 | 78 | 100 | 97 | 97 |
| gyro_bias=min | 97 | 100 | 88 | 94 | 94 | 100 |
| gyro_bias=max | 88 | 100 | 88 | 100 | 100 | 100 |
| accel_noise_std=min | 97 | 100 | 97 | 100 | 100 | 100 |
| accel_noise_std=max | 97 | 100 | 94 | 100 | 94 | 97 |
| vel_noise_std=min | 97 | 100 | 91 | 100 | 100 | 100 |
| vel_noise_std=max | 88 | 100 | 88 | 100 | 97 | 94 |
| pos_noise_std=min | 97 | 100 | 88 | 100 | 94 | 100 |
| pos_noise_std=max | 91 | 97 | 91 | 100 | 100 | 97 |

- v3: worst extreme `latency_s=max/tilt_60` -> 78%
- PID: worst extreme `motor_tau=max/throw` -> 88%
- nf3d: worst extreme `latency_s=max/tilt_60` -> 75%

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

| scenario | succ% v3 | succ% PID | succ% nf3d | v3 err_med m | PID err_med m | nf3d err_med m | v3 worst/p95 tilt° | PID worst/p95 tilt° | nf3d worst/p95 tilt° | v3 settle s | PID settle s | nf3d settle s | v3 trip% | PID trip% | nf3d trip% | v3 sup% | PID sup% | nf3d sup% | v3 HF rate/thr | PID HF rate/thr | nf3d HF rate/thr | v3 thr/PID | nf3d thr/PID |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover_1m/nominal | 94.5 | 99.2 | 4.3 | 0.088 | 0.019 | 0.113 | 37/33 | 39/37 | 33/28 | 1.11 | 0.91 | 0.99 | 0.0 | 0.0 | 0.0 | 0.0 | 2.7 | 0.0 | 0.0015/0.0012 | 0.0009/0.0004 | 0.0012/0.0006 | 2.73 | 1.30 |
| hover_1m/randomized | 50.0 | 98.0 | 35.2 | 0.099 | 0.029 | 0.117 | 36/33 | 42/38 | 35/29 | 1.03 | 0.95 | 1.07 | 0.0 | 0.0 | 0.0 | 0.0 | 5.5 | 0.0 | 0.0021/0.0017 | 0.0028/0.0022 | 0.0016/0.0014 | 0.79 | 0.66 |
| tilt_60/nominal | 49.2 | 98.0 | 17.6 | 0.100 | 0.028 | 0.122 | 60/60 | 60/60 | 60/60 | 0.93 | 0.79 | 1.03 | 0.0 | 0.0 | 0.0 | 13.3 | 2.7 | 10.2 | 0.0015/0.0004 | 0.0008/0.0002 | 0.0012/0.0003 | 2.59 | 1.61 |
| tilt_60/randomized | 33.6 | 96.5 | 29.7 | 0.115 | 0.035 | 0.125 | 60/60 | 60/60 | 60/60 | 0.89 | 0.86 | 1.14 | 0.0 | 0.0 | 0.0 | 8.6 | 0.4 | 5.5 | 0.0020/0.0013 | 0.0028/0.0023 | 0.0016/0.0014 | 0.59 | 0.62 |
| throw/nominal | 75.0 | 98.4 | 8.2 | 0.091 | 0.028 | 0.115 | 47/40 | 48/43 | 52/40 | 0.81 | 0.65 | 1.07 | 1.6 | 2.0 | 1.6 | 3.1 | 4.3 | 2.3 | 0.0017/0.0007 | 0.0012/0.0003 | 0.0014/0.0005 | 2.04 | 1.59 |
| throw/randomized | 54.3 | 96.9 | 34.4 | 0.097 | 0.032 | 0.115 | 52/41 | 61/44 | 46/39 | 0.67 | 0.62 | 0.76 | 2.3 | 4.3 | 2.3 | 3.1 | 5.5 | 2.3 | 0.0022/0.0016 | 0.0032/0.0023 | 0.0017/0.0014 | 0.67 | 0.61 |
| upside_down/nominal | 22.7 | 74.6 | 11.7 | 0.122 | 0.071 | 0.140 | 180/179 | 180/178 | 180/178 | 1.82 | 1.44 | 1.67 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0018/0.0019 | 0.0013/0.0010 | 0.0014/0.0019 | 1.87 | 1.88 |
| upside_down/randomized | 22.3 | 73.0 | 12.5 | 0.138 | 0.076 | 0.146 | 180/179 | 180/179 | 180/179 | 1.67 | 1.64 | 1.85 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0023/0.0024 | 0.0033/0.0025 | 0.0017/0.0027 | 0.98 | 1.06 |

Randomization extremes (one parameter at min/max, rest nominal, no other noise): success %

| param=extreme | v3 tilt_60 | PID tilt_60 | nf3d tilt_60 | v3 throw | PID throw | nf3d throw |
|---|---|---|---|---|---|---|
| mass=min | 81 | 97 | 59 | 100 | 100 | 91 |
| mass=max | 0 | 100 | 0 | 0 | 100 | 0 |
| inertia=min | 47 | 97 | 28 | 81 | 94 | 6 |
| inertia=max | 41 | 100 | 28 | 81 | 97 | 0 |
| thrust_max=min | 0 | 97 | 0 | 0 | 97 | 0 |
| thrust_max=max | 84 | 94 | 44 | 97 | 97 | 100 |
| per_motor_thrust_imbalance=min | 34 | 91 | 16 | 81 | 94 | 9 |
| per_motor_thrust_imbalance=max | 41 | 97 | 12 | 59 | 91 | 12 |
| yaw_torque_coeff=min | 53 | 100 | 12 | 66 | 88 | 6 |
| yaw_torque_coeff=max | 47 | 97 | 22 | 72 | 88 | 9 |
| motor_tau=min | 66 | 100 | 16 | 78 | 100 | 6 |
| motor_tau=max | 34 | 94 | 16 | 72 | 88 | 9 |
| lin_drag=min | 44 | 97 | 28 | 84 | 100 | 9 |
| lin_drag=max | 56 | 100 | 22 | 78 | 97 | 6 |
| latency_s=min | 44 | 91 | 6 | 88 | 100 | 12 |
| latency_s=max | 53 | 100 | 6 | 56 | 94 | 6 |
| gyro_noise_std=min | 53 | 97 | 6 | 75 | 97 | 9 |
| gyro_noise_std=max | 53 | 100 | 19 | 78 | 97 | 16 |
| gyro_bias=min | 56 | 100 | 16 | 72 | 88 | 9 |
| gyro_bias=max | 72 | 100 | 25 | 72 | 100 | 16 |
| accel_noise_std=min | 41 | 100 | 3 | 72 | 100 | 6 |
| accel_noise_std=max | 69 | 100 | 12 | 88 | 97 | 6 |
| vel_noise_std=min | 50 | 97 | 12 | 78 | 97 | 3 |
| vel_noise_std=max | 47 | 94 | 31 | 62 | 97 | 16 |
| pos_noise_std=min | 53 | 100 | 3 | 88 | 100 | 9 |
| pos_noise_std=max | 38 | 100 | 22 | 91 | 97 | 38 |

- v3: worst extreme `mass=max/tilt_60` -> 0%
- PID: worst extreme `yaw_torque_coeff=min/throw` -> 88%
- nf3d: worst extreme `mass=max/tilt_60` -> 0%

## Airframe gate

PASS

## Learned-policy findings

- nominal: v3 tilt_60/randomized 85.9 % < reference v3 90.6 % - 3
- nominal: nf3d hover_1m/randomized 96.9 % < reference v3 100.0 % - 3
- nominal: nf3d tilt_60/randomized 77.0 % < reference v3 90.6 % - 3
- nominal: nf3d throw/randomized 93.4 % < reference v3 97.3 % - 3
- pessimistic: v3 hover_1m/randomized 50.0 % < reference v3 100.0 % - 3
- pessimistic: v3 tilt_60/randomized 33.6 % < reference v3 90.6 % - 3
- pessimistic: v3 throw/randomized 54.3 % < reference v3 97.3 % - 3
- pessimistic: nf3d hover_1m/randomized 35.2 % < reference v3 100.0 % - 3
- pessimistic: nf3d tilt_60/randomized 29.7 % < reference v3 90.6 % - 3
- pessimistic: nf3d throw/randomized 34.4 % < reference v3 97.3 % - 3

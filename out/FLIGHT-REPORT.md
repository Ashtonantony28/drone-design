# Flight validation (neuroflight-fc simulator, firmware sensing path `fw3`)

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

| scenario | succ% v3 | succ% PID | v3 err_med m | PID err_med m | v3 worst/p95 tilt° | PID worst/p95 tilt° | v3 settle s | PID settle s | v3 trip% | PID trip% | v3 sup% | PID sup% | v3 HF rate/thr | PID HF rate/thr | v3 thr/PID |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover_1m/nominal | 100.0 | 100.0 | 0.028 | 0.018 | 34/32 | 38/37 | 0.95 | 0.91 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0017/0.0011 | 0.0007/0.0003 | 3.25 |
| hover_1m/randomized | 100.0 | 96.9 | 0.040 | 0.030 | 33/32 | 42/39 | 0.98 | 0.96 | 0.0 | 1.6 | 0.0 | 3.1 | 0.0019/0.0019 | 0.0028/0.0019 | 0.99 |
| tilt_60/nominal | 89.1 | 100.0 | 0.052 | 0.032 | 60/60 | 60/60 | 0.80 | 0.80 | 0.0 | 0.0 | 4.7 | 4.7 | 0.0017/0.0012 | 0.0008/0.0001 | 9.62 |
| tilt_60/randomized | 87.5 | 98.4 | 0.067 | 0.038 | 60/60 | 60/60 | 0.85 | 0.83 | 0.0 | 0.0 | 12.5 | 9.4 | 0.0021/0.0017 | 0.0027/0.0016 | 1.07 |
| throw/nominal | 98.4 | 95.3 | 0.036 | 0.030 | 46/43 | 47/43 | 0.78 | 0.69 | 4.7 | 3.1 | 4.7 | 6.2 | 0.0019/0.0012 | 0.0011/0.0003 | 4.26 |
| throw/randomized | 96.9 | 96.9 | 0.043 | 0.033 | 42/34 | 44/40 | 0.58 | 0.62 | 0.0 | 0.0 | 0.0 | 3.1 | 0.0021/0.0017 | 0.0031/0.0017 | 1.04 |
| upside_down/nominal | 60.9 | 78.1 | 0.082 | 0.070 | 180/179 | 180/179 | 1.43 | 1.40 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0020/0.0020 | 0.0012/0.0007 | 2.78 |
| upside_down/randomized | 53.1 | 68.8 | 0.097 | 0.072 | 180/178 | 180/178 | 1.45 | 1.66 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0023/0.0024 | 0.0031/0.0019 | 1.30 |

Randomization extremes (one parameter at min/max, rest nominal, no other noise): success %

| param=extreme | v3 tilt_60 | PID tilt_60 | v3 throw | PID throw |
|---|---|---|---|---|
| mass=min | 100 | 100 | 100 | 100 |
| mass=max | 81 | 94 | 100 | 100 |
| inertia=min | 81 | 100 | 94 | 100 |
| inertia=max | 100 | 100 | 100 | 94 |
| thrust_max=min | 94 | 100 | 100 | 100 |
| thrust_max=max | 81 | 100 | 100 | 100 |
| per_motor_thrust_imbalance=min | 88 | 100 | 100 | 94 |
| per_motor_thrust_imbalance=max | 94 | 100 | 100 | 100 |
| yaw_torque_coeff=min | 100 | 94 | 100 | 100 |
| yaw_torque_coeff=max | 100 | 100 | 94 | 100 |
| motor_tau=min | 94 | 100 | 100 | 100 |
| motor_tau=max | 94 | 100 | 100 | 88 |
| lin_drag=min | 88 | 100 | 100 | 94 |
| lin_drag=max | 94 | 100 | 100 | 94 |
| latency_s=min | 94 | 100 | 100 | 94 |
| latency_s=max | 100 | 100 | 100 | 100 |
| gyro_noise_std=min | 100 | 100 | 100 | 94 |
| gyro_noise_std=max | 81 | 100 | 100 | 100 |
| gyro_bias=min | 94 | 100 | 100 | 94 |
| gyro_bias=max | 100 | 100 | 100 | 100 |
| accel_noise_std=min | 94 | 94 | 94 | 100 |
| accel_noise_std=max | 81 | 100 | 100 | 100 |
| vel_noise_std=min | 100 | 100 | 100 | 94 |
| vel_noise_std=max | 88 | 100 | 100 | 94 |
| pos_noise_std=min | 94 | 100 | 100 | 94 |
| pos_noise_std=max | 100 | 100 | 100 | 94 |

- v3: worst extreme `mass=max/tilt_60` -> 81%
- PID: worst extreme `motor_tau=max/throw` -> 88%

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

| scenario | succ% v3 | succ% PID | v3 err_med m | PID err_med m | v3 worst/p95 tilt° | PID worst/p95 tilt° | v3 settle s | PID settle s | v3 trip% | PID trip% | v3 sup% | PID sup% | v3 HF rate/thr | PID HF rate/thr | v3 thr/PID |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover_1m/nominal | 95.3 | 100.0 | 0.086 | 0.016 | 33/32 | 38/37 | 1.10 | 0.90 | 0.0 | 0.0 | 0.0 | 1.6 | 0.0018/0.0012 | 0.0007/0.0005 | 2.59 |
| hover_1m/randomized | 64.1 | 100.0 | 0.092 | 0.032 | 34/32 | 39/39 | 1.08 | 0.95 | 0.0 | 0.0 | 0.0 | 4.7 | 0.0020/0.0019 | 0.0029/0.0025 | 0.76 |
| tilt_60/nominal | 46.9 | 100.0 | 0.100 | 0.027 | 60/60 | 60/60 | 0.91 | 0.81 | 0.0 | 0.0 | 9.4 | 0.0 | 0.0016/0.0004 | 0.0008/0.0002 | 2.80 |
| tilt_60/randomized | 40.6 | 96.9 | 0.107 | 0.034 | 60/60 | 60/60 | 0.91 | 0.84 | 0.0 | 0.0 | 4.7 | 0.0 | 0.0020/0.0013 | 0.0031/0.0022 | 0.60 |
| throw/nominal | 78.1 | 93.8 | 0.091 | 0.026 | 46/43 | 45/44 | 0.92 | 0.71 | 4.7 | 4.7 | 4.7 | 6.2 | 0.0018/0.0008 | 0.0011/0.0004 | 2.04 |
| throw/randomized | 56.2 | 98.4 | 0.094 | 0.028 | 40/33 | 47/40 | 0.63 | 0.64 | 0.0 | 0.0 | 0.0 | 4.7 | 0.0019/0.0013 | 0.0029/0.0022 | 0.59 |
| upside_down/nominal | 23.4 | 76.6 | 0.118 | 0.063 | 180/179 | 180/179 | 1.84 | 1.42 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0020/0.0020 | 0.0012/0.0010 | 2.02 |
| upside_down/randomized | 23.4 | 71.9 | 0.136 | 0.074 | 180/177 | 180/178 | 1.61 | 1.70 | 100.0 | 100.0 | 100.0 | 100.0 | 0.0025/0.0024 | 0.0033/0.0025 | 0.96 |

Randomization extremes (one parameter at min/max, rest nominal, no other noise): success %

| param=extreme | v3 tilt_60 | PID tilt_60 | v3 throw | PID throw |
|---|---|---|---|---|
| mass=min | 81 | 100 | 94 | 94 |
| mass=max | 0 | 100 | 0 | 100 |
| inertia=min | 44 | 100 | 81 | 94 |
| inertia=max | 56 | 100 | 88 | 94 |
| thrust_max=min | 0 | 94 | 0 | 100 |
| thrust_max=max | 88 | 88 | 100 | 100 |
| per_motor_thrust_imbalance=min | 38 | 100 | 88 | 100 |
| per_motor_thrust_imbalance=max | 31 | 94 | 81 | 100 |
| yaw_torque_coeff=min | 69 | 100 | 81 | 100 |
| yaw_torque_coeff=max | 56 | 100 | 69 | 100 |
| motor_tau=min | 56 | 100 | 94 | 100 |
| motor_tau=max | 38 | 100 | 56 | 81 |
| lin_drag=min | 50 | 100 | 69 | 94 |
| lin_drag=max | 44 | 100 | 94 | 100 |
| latency_s=min | 44 | 100 | 88 | 94 |
| latency_s=max | 38 | 100 | 75 | 94 |
| gyro_noise_std=min | 50 | 100 | 75 | 81 |
| gyro_noise_std=max | 62 | 100 | 69 | 94 |
| gyro_bias=min | 62 | 94 | 88 | 94 |
| gyro_bias=max | 50 | 100 | 94 | 94 |
| accel_noise_std=min | 38 | 100 | 62 | 100 |
| accel_noise_std=max | 69 | 100 | 62 | 100 |
| vel_noise_std=min | 38 | 100 | 75 | 88 |
| vel_noise_std=max | 19 | 100 | 62 | 94 |
| pos_noise_std=min | 44 | 100 | 88 | 100 |
| pos_noise_std=max | 56 | 94 | 62 | 100 |

- v3: worst extreme `mass=max/tilt_60` -> 0%
- PID: worst extreme `motor_tau=max/throw` -> 81%

## Result

FAIL
- nominal: PID hover_1m/randomized success 96.9 % < 99 %
- nominal: PID tilt_60/randomized success 98.4 % < 99 %
- nominal: PID throw/randomized success 96.9 % < 97 %
- nominal: v3 tilt_60/randomized success 87.5 % < 90 %
- pessimistic: PID tilt_60/randomized success 96.9 % < 99 %
- pessimistic: v3 hover_1m/randomized success 64.1 % < 95 %
- pessimistic: v3 tilt_60/randomized success 40.6 % < 90 %
- pessimistic: v3 throw/randomized success 56.2 % < 90 %

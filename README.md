# Tremor Orthosis — Active wrist orthosis for Parkinsonian tremor

**Joseph Belamich & Louis Gosset** · EPFL Mechanical Engineering

An active 1-DOF wrist orthosis that suppresses Parkinsonian tremor while leaving voluntary motion and the fingers free. Two IMUs measure the wrist motion; an ESP32 separates slow voluntary movement (< 2 Hz) from tremor (4–7 Hz) with a discrete Kalman filter at 500 Hz, and an LQR controller applies a counter-torque through Dyneema cables driven by two miniature brushless motors.

> **Status — work in progress (Phase 1, October 2026).** The firmware skeleton and interface contracts are in place. No experimental result has been validated yet; results will be added here as they are measured.

## Targets

| Metric | Target |
| --- | --- |
| RMS tremor attenuation (4–6 Hz) | > 50 % |
| Sensor-to-torque latency | < 20 ms |
| Slowing of voluntary motion (0.5 Hz) | < 10 % |

## Architecture

```
 IMU1 (forearm) ─┐                                         ┌─ Motor A (palmar cable, flexion)
                 ├─ I²C 400 kHz ─► ESP32, 500 Hz loop ─────┤
 IMU2 (hand)  ───┘     Kalman filter → LQR → torque split  └─ Motor B (dorsal cable, extension)
                                                                    │
                       forearm cuff (PETG) ── cables ── hand shell on a wrist pivot
```

- **Sensing** — two MPU-6050 (0x68 forearm, 0x69 hand); the relative gyro rate gives the wrist angular velocity.
- **Estimation** — state `x = [θ_v, ω_v, θ_t, ω_t]`: voluntary motion as a random walk, tremor as a discrete harmonic oscillator at the estimated frequency.
- **Control** — discrete LQR on the tremor states only, gain-scheduled on the tremor frequency.
- **Actuation** — two BLDC 2804 gimbal motors (SimpleFOC, torque mode), pulley-to-lever ratio ≈ 6, antagonistic cables with pretension.
- **Safety** — software torque limit, mechanical stops at ±40°, loop and IMU watchdogs, hard-wired emergency stop, motors disabled at boot.

## Repository layout

```
include/contracts.h     interface contracts: pinout, units, signs, firmware API, safety limits
src/main.cpp            500 Hz loop, watchdog, serial commands, CSV logging
src/drivers/            IMU, motors, power (mock motors for development)
src/estimator/          Kalman filter, frequency estimator
src/control/            controller_step (LQR) and open-loop test stub
matlab/                 modelling, Kalman/LQR design, analysis and figures
data/ref/               reference logs used for regression tests
docs/                   design notes
test/                   host-side tests (C++ vs MATLAB equivalence)
```

## Build and run

Requires [PlatformIO](https://platformio.org/).

```bash
pio run -e mock -t upload      # IMUs + controller, motors mocked
pio run -e openloop -t upload  # sine torque, for mechanical tests
pio device monitor             # 921600 baud
```

Serial commands: `ARM`, `DISARM`, `LOG on`, `LOG off`, `SET tau_max 0.1`, `STATUS`.
The log is one CSV line per sample (`t_us, ax1 … gz2, theta_hat, theta_v_hat, theta_t_hat, f_hat, tau_cmd, vbat`).

## Safety note

This is a research prototype tested on a bench and on healthy volunteers simulating tremor, at reduced torque. It is not a medical device and has not been tested on patients.

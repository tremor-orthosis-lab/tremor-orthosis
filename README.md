# Tremor Orthosis — Active wrist orthosis for Parkinsonian tremor

**Joseph Belamich & Louis Gosset** · EPFL Mechanical Engineering

An active 1-DOF wrist orthosis that suppresses Parkinsonian tremor while leaving voluntary motion and the fingers free. Two IMUs measure the wrist motion; an ESP32 separates slow voluntary movement (< 2 Hz) from tremor (4–7 Hz) with a discrete Kalman filter at 500 Hz, and an LQR controller applies a counter-torque through Dyneema cables driven by two miniature brushless motors.

> **Status — work in progress (Phase 1, October 2026).** The firmware skeleton, interface contracts and the tremor estimator are in place. The estimator is validated on synthetic data only (MATLAB, and C++ against MATLAB); no experimental result has been validated yet. Results will be added here as they are measured.

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
- **Estimation** — state `x = [θ_v, ω_v, θ_t, ω_t]`: voluntary motion as a constant-velocity model driven by white acceleration, tremor as a damped discrete harmonic oscillator. Its frequency is tracked online by a weighted-frequency Fourier linear combiner (WFLC) on the relative gyro rate, which keeps the phase lag of the tremor estimate below 5° when the tremor frequency drifts between 4 and 6 Hz (synthetic data).
- **Control** — torque proportional to the estimated tremor velocity only (a "virtual damper" that leaves voluntary motion free), gain-scheduled on the estimated tremor frequency. Designed with the Kalman filter and the 6 ms delay in the loop: a textbook full-state LQR with an internal tremor model becomes unstable once it is fed with the Kalman estimate. Simulation (nominal model): 55–62 % tremor attenuation at 4–6 Hz, voluntary motion at 0.5 Hz changed by less than 8 %, stable for ±30–50 % model errors.
- **Actuation** — two DFRobot 4015 gimbal BLDC motors with integrated AS5600 encoders (SimpleFOC, torque mode), pulley-to-lever ratio ≈ 3, antagonistic cables with pretension.
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
docs/                   wiring and first power-up, recording protocol
tools/                  record_log.py: record a trial from the serial port
test/                   host-side tests (C++ vs MATLAB equivalence)
```

## Build and run

Requires [PlatformIO](https://platformio.org/).

```bash
pio run -e mock -t upload      # IMUs + controller, motors mocked
pio run -e openloop -t upload  # sine torque, for mechanical tests
pio device monitor             # 921600 baud
```

## Tests

The estimator in `src/estimator/` is a line-by-line single-precision port of the MATLAB reference in `matlab/`. A host-side test checks that both give the same outputs on the reference data in `data/ref/` (any C++17 compiler, no hardware needed):

```bash
g++ -std=c++17 -O2 -Isrc/estimator test/host/test_estimator.cpp src/estimator/tremor_estimator.cpp -o test_estimator && ./test_estimator
```

Serial commands: `SCAN`, `WATCH on/off`, `LOG on/off`, `STATUS`, `ARM`, `DISARM`, `SET tau_max 0.1`.
The log is one CSV line per sample (`n, t_us, ax1 … gz2, theta_hat, theta_v_hat, theta_t_hat, f_hat, tau_cmd, vbat, loop_us`); `n` is a sample counter to detect lost lines.

First power-up of the sensor kit: [`docs/bringup.md`](docs/bringup.md). Recording a trial: `python3 tools/record_log.py trem5 -s 30` (protocol: [`docs/recording_protocol.md`](docs/recording_protocol.md)); analysis in MATLAB: `analyze_log('../data/raw/<file>.csv')`.

## Safety note

This is a research prototype tested on a bench and on healthy volunteers simulating tremor, at reduced torque. It is not a medical device and has not been tested on patients.

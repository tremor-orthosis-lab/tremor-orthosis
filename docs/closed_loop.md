# Identification and first closed loop (tasks J-3.3, J-3.4)

For the test bench (wrist emulator) first, with motors and drivers working (L-2.3), then on a healthy wrist at reduced torque. Never on a patient.

## Safety rules

- Start every session with `SET tau_max 0.05`. Raise step by step (0.1, then 0.2) only after a clean test.
- Keep a hand on the emergency stop. `DISARM` cuts the torque immediately.
- Motors are off at boot. The controller (`CTRL`) is off at boot. Torque is applied only after `ARM`.
- One change at a time; log every trial.

## 1. Identification (J-3.3): what the real wrist looks like

The controller gains were designed on the nominal model of `contracts.h` (C4). This step measures the real inertia, stiffness, damping and delay.

1. Hand (or bench hand) relaxed, not resisting.
2. Keep the arm still at start-up (gyro calibration).
3. Close the PlatformIO monitor and record; the recorder resets the board, then sends the commands once the log has started, and always ends with `DISARM`:
   ```bash
   python3 tools/record_log.py ident -s 35 -c "SET tau_max 0.1" -c ARM -c "IDENT 0.05 0.5 12 30"
   ```
   (Chirp: 0.05 N·m, 0.5 → 12 Hz in 30 s. If the coherence is too low, try 0.08 N·m.)
4. In MATLAB (`matlab/` folder):
   ```matlab
   id = identify('../data/raw/<date>_<time>_ident.csv');
   ```
   It prints J, K, B and the delay and shows the measured vs fitted Bode plot. The fit is good if the red curve follows the black dots.
5. Put the values in `tremor_params.m` (`p.plant`, `delay_steps`), run `design_controller` and `sim_closed_loop`, and copy the new gains into `include/contracts.h` (`Gains::K`).

Tested on simulated logs (`gen_fake_ident_log.m`): J, K, B recovered within 1–4 %, delay within 1 ms.

## 2. First closed loop (J-3.4)

1. Tremor source running (bench motor at 5 Hz, or simulated tremor). In the PlatformIO monitor (opening it resets the board: arm still):
   ```
   SET tau_max 0.05
   CTRL on
   WATCH on
   ```
   Check that `f` is close to the tremor frequency and that `tau` oscillates.
2. `ARM`. The gain ramps in over 1 s.
3. **The tremor must DECREASE within 1–2 s.** If it increases, `DISARM` immediately: a sign is wrong somewhere (IMU mounting or motor direction). Do not "fix" it by flipping a gain — find which sign is wrong (C2).
4. Recorded trials, passive then active with the same tremor:
   ```bash
   python3 tools/record_log.py bench5_passive -s 30 -c "SET tau_max 0.1"
   python3 tools/record_log.py bench5_active  -s 30 -c "SET tau_max 0.1" -c "CTRL on" -c ARM
   ```

## What is already checked without hardware

`test/host/test_closed_loop.cpp` runs the real firmware controller on a simulated wrist, with simulated IMUs in the C2 frames (including the hand acceleration seen by the accelerometer). It passes only if the tremor is attenuated by more than 50 % at 4, 5 and 6 Hz and the voluntary motion changes by less than 10 %:

```bash
g++ -std=c++17 -O2 -Iinclude -Isrc/estimator test/host/test_closed_loop.cpp src/control/controller.cpp src/estimator/tremor_estimator.cpp -o test_closed_loop && ./test_closed_loop
```

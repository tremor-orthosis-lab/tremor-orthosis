# Recording protocol on yourself (task J-1.3)

Goal: at least 8 clean recordings of voluntary motion and simulated tremor, to check the Kalman filter, the WFLC and the accelerometer angle on real data. About 1 h 30, after the kit passes the test of `docs/bringup.md`.

## Setup

- IMUs mounted as in `docs/bringup.md` §2 (X arrows to the fingers, components facing out).
- Sit at a table, **forearm resting on the table**, hand free beyond the edge, palm down. The forearm stays still for all trials except "glass".
- Phone on a stand, side view of the wrist, **slow motion 240 fps**: start the video before the recording and clap once (the clap shows on the IMUs too, to synchronise).
- Metronome app in the earphones. One beat = one full cycle (up + down):
  4 Hz = **240 bpm**, 5 Hz = **300 bpm**, 6 Hz = **360 bpm**.
- Close the PlatformIO monitor. One terminal in the repository folder.

## Trials

For each trial: `python3 tools/record_log.py <name> -s <seconds>`, press Enter, perform the task. The file goes to `data/raw/<date>_<time>_<name>.csv`.

| # | name | s | What to do |
| --- | --- | --- | --- |
| 1 | `rest` | 60 | hand relaxed and still |
| 2 | `vol` | 60 | slow flexion–extension, ±20–30°, about one cycle every 2 s (0.3–1 Hz), smooth |
| 3 | `trem4` | 30 | small fast oscillation (±3–5°) on the 240 bpm metronome |
| 4 | `trem5` | 30 | same at 300 bpm |
| 5 | `trem6` | 30 | same at 360 bpm (hard: 20 s is enough) |
| 6 | `trem5vol` | 30 | 5 Hz oscillation **while** doing the slow motion of trial 2 |
| 7 | `glass` | 60 | pick up a glass, bring it to the mouth, put it down, ×5 (forearm moves too) |
| 8 | `trem5b` | 30 | trial 4 again, IMU straps tightened (repeatability) |

Simulated tremor is tiring: rest 1 min between trials. If the rhythm breaks, redo the trial rather than keep a bad one.

## After each trial (1 min)

In MATLAB, in the `matlab/` folder:

```matlab
A = analyze_log('../data/raw/20261022_183000_trem5.csv');
```

Check:

- **0 lost samples**, 500 Hz.
- Tremor trials: peak within **±0.2 Hz** of the metronome (J-1.4 criterion), WFLC `f_hat` close to it.
- Write one line per trial in the notes: name, what happened, anything odd.

## The numbers to bring back (the open question)

`analyze_log` prints **"Accelerometer angle in the tremor band vs gyro: gain …, lag …"**.

- Gain ≈ 1: the accelerometer angle is clean.
- Gain ≫ 1 (simulation with the IMU 2 cm above the wrist axis gives ≈ 3): the hand acceleration corrupts it. The Kalman filter still works because it relies on the gyro (`r_theta_scale = 25`), but `r_theta_scale` will be re-tuned on these logs.

Also note the estimator line: correlation and lag of the tremor estimate vs the reference.

## At the end

- Copy all of `data/raw/` to the Drive, folder "Logs J-1.3", with the notes.
- Copy 3 files into `data/ref/` for regression tests: `rest`, `trem5`, `trem5vol`, renamed `real_rest.csv`, `real_trem5.csv`, `real_trem5vol.csv`. Commit them.

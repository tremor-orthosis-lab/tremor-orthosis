# MATLAB

Modelling, estimator and controller design, and analysis of the logs.

Planned scripts:

| Script | Purpose |
| --- | --- |
| `load_log.m` | read a CSV log (format C3) into a struct |
| `psd_analysis.m` | Welch PSD and spectrogram, tremor peak in 3–8 Hz |
| `kalman_design.m` | state model, Kalman filter written by hand, Q/R tuning |
| `plant_model.m` | wrist + orthosis model, parameters from `contracts.h` (C4) |
| `lqr_design.m` | discrete LQR, gain table for f0 = 4, 5, 6 Hz, Monte-Carlo robustness |
| `identify.m` | identification of J, b, k, gain and delay from `MODE ident` logs |
| `metrics.m` | RMS attenuation, PSD reduction, latency, figures |

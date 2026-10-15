# MATLAB

Modelling, estimator and controller design, and analysis of the logs. No toolbox needed (base MATLAB; the scripts also run in GNU Octave without the figures). All parameters are in `tremor_params.m`.

## Scripts to run

| Script | Task | What it does |
| --- | --- | --- |
| `run_kalman_synthetic.m` | J-1.5 | Kalman filter on synthetic data vs band-pass; exports `data/ref/kalman_ref.csv` |
| `run_adaptive_synthetic.m` | J-4.1 | online tremor-frequency estimation (WFLC vs EKF); exports `data/ref/wflc_ref.csv` |
| `design_controller.m` | J-2.3 | controller design with the Kalman filter in the loop, Monte-Carlo robustness |
| `sim_closed_loop.m` | J-2.2, J-2.3 | time-domain closed loop (noise, WFLC, saturation, delay), torque needed |

## Functions

| Function | Purpose |
| --- | --- |
| `tremor_params.m` | all parameters: estimator, plant (C4), controller gains |
| `gen_synthetic.m` | synthetic voluntary motion + tremor + sensor noise |
| `tremor_kalman.m` | Kalman filter, reference for the C++ port (`src/estimator/`) |
| `wflc_freq.m` | WFLC frequency estimator, reference for the C++ port |
| `tremor_ekf_freq.m` | alternative: EKF with the frequency as a state (comparison only) |
| `estimator_step.m` | WFLC + Kalman one sample at a time, for closed-loop simulation |
| `plant_model.m` | wrist + orthosis model (J-2.1) |
| `kalman_steady.m`, `closed_loop_matrix.m`, `loop_metrics.m` | linear analysis of plant + Kalman + delay + controller |
| `dlqr_iter.m` | discrete LQR by Riccati iteration |
| `inverse_dynamics.m` | torque that produces a given free wrist motion |
| `bandpass_biquad.m`, `phase_fit.m`, `phase_lag_windowed.m` | baseline filter and phase-lag metrics |

## Still to write

| Script | Task |
| --- | --- |
| `load_log.m`, `psd_analysis.m` | J-1.4, read and analyse the real logs |
| `identify.m` | J-3.4, identification of J, b, k and delay |
| `metrics.m` | J-4.2, metrics pipeline on the bench logs |

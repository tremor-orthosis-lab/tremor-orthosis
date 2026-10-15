function A = analyze_log(path, show)
%ANALYZE_LOG One-command analysis of a recorded trial (J-1.3 / J-1.4).
%   A = analyze_log('data/raw/20261022_183000_trem5.csv')
%   1. Checks the recording (lost samples, rate, loop time).
%   2. Spectrum of the wrist angle, tremor peak (psd_analysis.m).
%   3. Accelerometer check: in the tremor band, does the accelerometer
%      angle agree with the gyro? (gain 1, lag 0 = no corruption by the
%      acceleration of the hand). Tells how much r_theta must be inflated.
%   4. Runs the estimator offline (wflc_freq + tremor_kalman) on the
%      measurements and compares it with the reference tremor (zero-phase
%      band-pass 2.5-9 Hz of the reference angle). If the firmware was
%      running the estimator, also compares firmware and MATLAB outputs.
if nargin < 2, show = true; end
p = tremor_params();
L = load_log(path);
Ts = 1/L.fs; t = L.t; d = 180/pi;
skip = t > 2;                                  % ignore the first 2 s

fprintf('\n=== %s ===\n', L.name);
fprintf('Recording: %.1f s, %.1f Hz, %d lost samples, worst loop %d us\n', ...
        t(end), L.fs, L.lost, max(L.loop_us));
if L.lost > 0 || abs(L.fs - 500) > 2
    fprintf('  WARNING: lost samples or wrong rate, see docs/bringup.md\n');
end

R = psd_analysis(L, show);
fprintf('Tremor peak (3-8 Hz): %.2f Hz, amplitude %.2f deg\n', R.f_peak, R.amp_peak);
if R.amp_peak < 0.3
    fprintf('  (below 0.3 deg: no real tremor in this trial, the peak is noise)\n');
end

bp = @(x) flipud(bandpass_biquad(flipud(bandpass_biquad(x, 2.5, 9, Ts)), 2.5, 9, Ts));
ref = bp(R.theta_ref);
ref_rms = std(ref(skip));

% accelerometer angle vs gyro in the tremor band
gyro_angle = bp(cumsum(L.omega_mes)*Ts);
acc_angle  = bp(L.theta_mes);
[lag_acc, gain_acc] = phase_fit(acc_angle(skip), gyro_angle(skip), R.f_peak, t(skip));
fprintf('Accelerometer angle in the tremor band vs gyro: gain %.2f, lag %+.0f deg\n', ...
        gain_acc, lag_acc);
fprintf('  (gain 1 and lag 0 = clean; otherwise the hand acceleration corrupts it)\n');

% estimator offline
[f_w, act] = wflc_freq(L.omega_mes, p);
if any(skip & act), f_med = median(f_w(skip & act)); else, f_med = NaN; end
X = tremor_kalman([L.theta_mes, L.omega_mes], f_w, p);
est = X(:,3);
r = corrcoef(est(skip), ref(skip));
[lag_est, gain_est] = phase_fit(est(skip), ref(skip), R.f_peak, t(skip));
fprintf('Estimator (WFLC + Kalman, offline): f_hat median %.2f Hz (adapting %.0f %% of the time)\n', ...
        f_med, 100*mean(act(skip)));
fprintf('  tremor estimate vs reference: corr %.3f, lag %+.1f deg, gain %.2f\n', r(1,2), lag_est, gain_est);

% firmware vs MATLAB (only if the firmware estimator was running)
if any(L.theta_t_hat ~= 0)
    late = t > 5;
    dmax = max(abs(L.theta_t_hat(late) - est(late)))*d;
    fprintf('Firmware vs MATLAB theta_t (after 5 s): max difference %.3f deg\n', dmax);
end

A = struct('name', L.name, 'f_peak', R.f_peak, 'amp_deg', R.amp_peak, ...
           'lost', L.lost, 'fs', L.fs, 'acc_gain', gain_acc, 'acc_lag', lag_acc, ...
           'est_corr', r(1,2), 'est_lag', lag_est, 'f_hat', f_med, ...
           'ref_rms_deg', ref_rms*d);

if show
    figure('Name', ['Estimator ' L.name], 'Color', 'w');
    subplot(2,1,1);
    plot(t, ref*d, 'k', t, est*d, 'r--'); grid on;
    ylabel('\theta_t [deg]'); legend('reference (zero-phase band-pass)', 'Kalman + WFLC');
    xl = min(t(end), 6); xlim([xl-2 xl]);
    title(strrep(L.name, '_', '\_'));
    subplot(2,1,2);
    plot(t, f_w, 'r'); hold on; yline(R.f_peak, 'k:'); grid on;
    ylim([2 9]); xlabel('time [s]'); ylabel('f [Hz]'); legend('WFLC', 'PSD peak');
end
end

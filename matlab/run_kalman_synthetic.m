
%RUN_KALMAN_SYNTHETIC  Task J-1.5: validate the tremor Kalman filter on
% synthetic data and compare it with a band-pass filter.
%
% What it does
%   1. Generates voluntary motion + tremor at 4, 5 and 6 Hz (+ noise).
%   2. Runs the Kalman filter (a) with the model frequency fixed at 5 Hz,
%      (b) with the model frequency equal to the true tremor frequency
%      (what the adaptive estimator of J-4.1 will provide).
%   3. Runs a 3-8 Hz band-pass filter as the baseline.
%   4. Prints correlation, phase lag, gain and voluntary leakage.
%   5. Plots the results and exports data/ref/kalman_ref.csv for the
%      C++ equivalence test (task J-2.4).
%
% Success criteria from the plan (synthetic data):
%   correlation > 0.95, phase lag at 5 Hz < 10 deg, leakage < 10 %.

clear; clc; close all;
here = fileparts(mfilename('fullpath'));
p  = tremor_params();
Ts = p.Ts;  T = 30;  skip = round(2/Ts);   % ignore the first 2 s (transient)
freqs = [4 5 6];

fprintf('%-24s %5s %8s %10s %6s %8s\n', 'Estimator', 'f', 'corr', 'lag [deg]', 'gain', 'leak');
results = struct([]);
for f = freqs
    s  = gen_synthetic(f, T, Ts, 1);
    s0 = gen_synthetic(0, T, Ts, 2);          % voluntary only, for leakage
    idx = skip+1:numel(s.t);
    tt  = s.t(idx);  ref = s.theta_t(idx);
    volstd = std(s0.theta_v(idx));

    % (a) model frequency fixed at 5 Hz
    Xa  = tremor_kalman(s.y, 5, p);
    X0a = tremor_kalman(s0.y, 5, p);
    % (b) model frequency = true frequency
    Xb  = tremor_kalman(s.y, f, p);
    X0b = tremor_kalman(s0.y, f, p);
    % (c) band-pass on the measured angle
    bp  = bandpass_biquad(s.y(:,1), 3, 8, Ts);
    bp0 = bandpass_biquad(s0.y(:,1), 3, 8, Ts);

    est  = {Xa(idx,3), Xb(idx,3), bp(idx)};
    est0 = {X0a(idx,3), X0b(idx,3), bp0(idx)};
    names = {'Kalman, f0 = 5 Hz', 'Kalman, f0 = true f', 'Band-pass 3-8 Hz'};
    for e = 1:3
        r = corrcoef(est{e}, ref);
        [lag, g] = phase_fit(est{e}, ref, f, tt);
        leak = std(est0{e})/volstd;
        fprintf('%-24s %4d %8.3f %10.1f %6.2f %7.1f%%\n', names{e}, f, r(1,2), lag, g, 100*leak);
        results(end+1).name = names{e}; %#ok<SAGROW>
        results(end).f = f; results(end).corr = r(1,2);
        results(end).lag = lag; results(end).gain = g; results(end).leak = leak;
    end
    fprintf('\n');

    if f == 5
        s5 = s; X5 = Xa; bp5 = bp;
    end
end

%% Figure 1: time series at 5 Hz
d = 180/pi;
figure('Name','Synthetic test at 5 Hz','Color','w');
tl = tiledlayout(3,1,'TileSpacing','compact');
nexttile; plot(s5.t, d*s5.y(:,1), 'Color', [0.7 0.7 0.7]); hold on;
plot(s5.t, d*(s5.theta_v + s5.theta_t), 'k');
ylabel('\theta [deg]'); legend('measured', 'true total'); xlim([2 8]);
nexttile; plot(s5.t, d*s5.theta_v, 'k'); hold on; plot(s5.t, d*X5(:,1), 'r--');
ylabel('\theta_v [deg]'); legend('true voluntary', 'Kalman'); xlim([2 8]);
nexttile; plot(s5.t, d*s5.theta_t, 'k'); hold on; plot(s5.t, d*X5(:,3), 'r--');
plot(s5.t, d*bp5, 'b:');
ylabel('\theta_t [deg]'); xlabel('time [s]');
legend('true tremor', 'Kalman', 'band-pass 3-8 Hz'); xlim([2 3]);
title(tl, 'Kalman separation of voluntary motion and tremor (synthetic, 5 Hz)');

%% Figure 2: phase lag versus frequency
figure('Name','Phase lag','Color','w');
for e = 1:3
    sel = strcmp({results.name}, names{e});
    plot([results(sel).f], [results(sel).lag], '-o', 'LineWidth', 1.5); hold on;
end
yline(10, 'k:'); yline(-10, 'k:');
xlabel('tremor frequency [Hz]'); ylabel('phase lag of the estimate [deg]');
legend(names, 'Location', 'best'); grid on;
title('Phase lag of the tremor estimate (target |lag| < 10 deg)');

%% Export reference vectors for the C++ port (J-2.4)
outdir = fullfile(here, '..', 'data', 'ref');
if ~exist(outdir, 'dir'), mkdir(outdir); end
ref_tbl = [s5.t, s5.y, X5];
fid = fopen(fullfile(outdir, 'kalman_ref.csv'), 'w');
fprintf(fid, 't,y_theta,y_omega,theta_v_hat,omega_v_hat,theta_t_hat,omega_t_hat\n');
fprintf(fid, '%.6f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n', ref_tbl');
fclose(fid);
fprintf('Exported %d samples to data/ref/kalman_ref.csv (f0 = 5 Hz, params from tremor_params.m)\n', size(ref_tbl,1));

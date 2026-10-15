%RUN_ADAPTIVE_SYNTHETIC  Task J-4.1: adaptive tremor-frequency estimation.
%
% Question
%   run_kalman_synthetic.m showed that the Kalman filter is near perfect
%   when its model frequency f0 equals the true tremor frequency, and lags
%   by 16-24 deg when it is off by 1 Hz. Real tremor frequency varies
%   (between patients, over minutes, with medication). Can we estimate f0
%   online and get the "oracle" performance?
%
% Estimators compared on 5 scenarios (40 s each)
%   KF 5 Hz   : Kalman filter, model frequency fixed at 5 Hz (J-1.5)
%   Oracle    : Kalman filter fed with the TRUE frequency (upper bound)
%   WFLC + KF : frequency estimated by wflc_freq.m, fed to the Kalman filter
%   EKF       : frequency as a 5th state of an extended Kalman filter
%
% Metrics (first 3 s ignored = convergence)
%   corr      : correlation of the tremor estimate with the true tremor
%   lag mean / lag max : |phase lag| over 1 s windows [deg]
%   f RMSE    : error of the frequency estimate [Hz]
%   leak      : on voluntary motion only, std(tremor estimate)/std(voluntary)
%               -> the share of voluntary motion the motor would fight
%
% Outputs: table in the Command Window, 3 figures, and
% data/ref/wflc_ref.csv (reference vectors for the C++ port of the WFLC).

clear; clc; close all;
here = fileparts(mfilename('fullpath'));
p  = tremor_params();
Ts = p.Ts;  T = 40;  skip = round(3/Ts);

% Slow random wander of the frequency around 5 Hz (std 0.4 Hz, ~1 s scale)
rng(7);
nz = filter(0.002, [1 -0.998], randn(round(T/Ts),1));
nz = 0.4*nz/std(nz);
wander = @(t) 5 + nz(1:numel(t));

scen = { ...
    'constant 5 Hz',        5,                          5; ...
    'chirp 4->6 Hz',        @(t) 4 + 2*t/T,             5; ...
    'chirp 4->6 Hz, 1 deg', @(t) 4 + 2*t/T,             1; ...
    'step 4.5->5.5 Hz',     @(t) 4.5 + (t >= 20),       2; ...
    'wander 5+/-0.4 Hz',    wander,                     5};
names = {'KF 5 Hz', 'Oracle', 'WFLC + KF', 'EKF'};

fprintf('%-22s %-10s %6s %9s %8s %8s\n', 'Scenario', 'Estimator', 'corr', 'lag mean', 'lag max', 'f RMSE');
res = struct([]);
for i = 1:size(scen,1)
    s = gen_synthetic(scen{i,2}, T, Ts, 1, scen{i,3});
    idx = skip+1:numel(s.t);

    [f_w, act] = wflc_freq(s.y(:,2), p);
    [Xe, f_e]  = tremor_ekf_freq(s.y, p);
    est = { tremor_kalman(s.y, 5, p), ...
            tremor_kalman(s.y, s.f_true, p), ...
            tremor_kalman(s.y, f_w, p), ...
            Xe };
    fh  = {[], [], f_w, f_e};

    for e = 1:4
        th = est{e}(idx,3);  ref = s.theta_t(idx);
        r  = corrcoef(th, ref);
        L  = phase_lag_windowed(th, ref, s.f_true(idx), s.t(idx));
        if isempty(fh{e}), frm = NaN;
        else, frm = sqrt(mean((fh{e}(idx) - s.f_true(idx)).^2)); end
        fprintf('%-22s %-10s %6.3f %9.1f %8.1f %8.2f\n', scen{i,1}, names{e}, ...
                r(1,2), mean(abs(L)), max(abs(L)), frm);
        res(end+1).scen = scen{i,1}; %#ok<SAGROW>
        res(end).est = names{e}; res(end).corr = r(1,2);
        res(end).lagmean = mean(abs(L)); res(end).lagmax = max(abs(L));
        res(end).frmse = frm;
    end
    fprintf('\n');

    runs(i).s = s; runs(i).f_w = f_w; runs(i).f_e = f_e; runs(i).act = act; %#ok<SAGROW>
    runs(i).X5 = est{1}; runs(i).Xw = est{3};
end

%% Leakage: voluntary motion only, no tremor
s0 = gen_synthetic(0, T, Ts, 2);
idx = skip+1:numel(s0.t);
volstd = std(s0.theta_v(idx));
[f_w0, act0] = wflc_freq(s0.y(:,2), p);
[Xe0, f_e0]  = tremor_ekf_freq(s0.y, p);
X50 = tremor_kalman(s0.y, 5, p);
Xw0 = tremor_kalman(s0.y, f_w0, p);
fprintf('Leakage (voluntary motion only)\n');
fprintf('  KF 5 Hz   : %4.1f %%\n', 100*std(X50(idx,3))/volstd);
fprintf('  WFLC + KF : %4.1f %%   (WFLC adapting %.0f %% of the time)\n', ...
        100*std(Xw0(idx,3))/volstd, 100*mean(act0(idx)));
fprintf('  EKF       : %4.1f %%   (EKF frequency drifts between %.1f and %.1f Hz)\n', ...
        100*std(Xe0(idx,3))/volstd, min(f_e0(idx)), max(f_e0(idx)));

%% Figure 1: frequency tracking
figure('Name','Frequency tracking','Color','w');
tl = tiledlayout(2,2,'TileSpacing','compact');
for i = [2 3 4 5]
    nexttile; s = runs(i).s;
    plot(s.t, s.f_true, 'k', 'LineWidth', 1.5); hold on;
    plot(s.t, runs(i).f_w, 'r'); plot(s.t, runs(i).f_e, 'b:');
    ylim([3 7]); grid on; title(scen{i,1});
    if i >= 4, xlabel('time [s]'); end
    ylabel('f [Hz]');
end
legend('true', 'WFLC', 'EKF', 'Location', 'best');
title(tl, 'Online estimate of the tremor frequency');

%% Figure 2: phase lag over time, chirp 4 -> 6 Hz
s = runs(2).s; idx = skip+1:numel(s.t);
[L5, tc] = phase_lag_windowed(runs(2).X5(idx,3), s.theta_t(idx), s.f_true(idx), s.t(idx));
Lw       = phase_lag_windowed(runs(2).Xw(idx,3), s.theta_t(idx), s.f_true(idx), s.t(idx));
figure('Name','Phase lag, chirp','Color','w');
plot(tc, L5, '-o', tc, Lw, '-s', 'LineWidth', 1.5); hold on;
yline(10, 'k:'); yline(-10, 'k:'); grid on;
xlabel('time [s]  (tremor frequency goes from 4 to 6 Hz)');
ylabel('phase lag of the tremor estimate [deg]');
legend('KF, f0 fixed at 5 Hz', 'KF + WFLC (adaptive)', 'Location', 'best');
title('Phase lag during a 4 \rightarrow 6 Hz chirp (target |lag| < 10 deg)');

%% Figure 3: time series at the end of the chirp (f ~ 5.9 Hz)
d = 180/pi;
figure('Name','Tremor estimate, chirp','Color','w');
plot(s.t, d*s.theta_t, 'k', 'LineWidth', 1.5); hold on;
plot(s.t, d*runs(2).X5(:,3), 'b--'); plot(s.t, d*runs(2).Xw(:,3), 'r--');
xlim([37 38]); grid on; xlabel('time [s]'); ylabel('\theta_t [deg]');
legend('true tremor', 'KF, f0 = 5 Hz', 'KF + WFLC');
title('Tremor estimate at ~5.9 Hz: the fixed model lags, the adaptive one does not');

%% Export reference vectors for the C++ port of the WFLC (chirp scenario)
outdir = fullfile(here, '..', 'data', 'ref');
if ~exist(outdir, 'dir'), mkdir(outdir); end
fid = fopen(fullfile(outdir, 'wflc_ref.csv'), 'w');
fprintf(fid, 't,y_theta,y_omega,f_hat,active,theta_t_hat\n');
fprintf(fid, '%.6f,%.9g,%.9g,%.9g,%d,%.9g\n', ...
        [s.t, s.y, runs(2).f_w, double(runs(2).act), runs(2).Xw(:,3)]');
fclose(fid);
fprintf('\nExported %d samples to data/ref/wflc_ref.csv (chirp 4->6 Hz)\n', numel(s.t));

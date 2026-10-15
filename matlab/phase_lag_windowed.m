function [lags, tc] = phase_lag_windowed(est, ref, f_true, t, win)
%PHASE_LAG_WINDOWED Phase lag of est vs ref on consecutive windows.
%   [lags, tc] = phase_lag_windowed(est, ref, f_true, t, win)
%   For a tremor whose frequency changes over time, a single global fit
%   (phase_fit.m) is meaningless. The signal is cut into windows of win
%   seconds (default 1 s); in each window the lag is fitted at the mean
%   true frequency. lags > 0 means est lags behind ref.
%   tc : centre time of each window.
if nargin < 5, win = 1.0; end
Ts = t(2) - t(1);
n  = round(win/Ts);
starts = 1:n:numel(t)-n+1;
lags = zeros(numel(starts),1);
tc   = zeros(numel(starts),1);
for i = 1:numel(starts)
    sl = starts(i):starts(i)+n-1;
    lags(i) = phase_fit(est(sl), ref(sl), mean(f_true(sl)), t(sl));
    tc(i) = mean(t(sl));
end
end

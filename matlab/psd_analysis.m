function R = psd_analysis(L, show)
%PSD_ANALYSIS Spectrum of the wrist motion in a log (task J-1.4).
%   R = psd_analysis(L, show)   L from load_log, show = true for the figure
%   Reference angle (offline, not the real-time estimator): complementary
%   filter, gyro integrated (gy1 - gy2) and drift corrected by the
%   accelerometer angle with a 0.5 s time constant.
%   Welch PSD (2 s Hann windows, 50 % overlap) and spectrogram, written by
%   hand (no Signal Processing Toolbox).
%   R.f, R.P       frequency [Hz] and PSD of the angle [deg^2/Hz]
%   R.f_peak       tremor peak in 3-8 Hz [Hz]
%   R.amp_peak     amplitude of the tremor [deg], from the power within
%                  +/-0.5 Hz of the peak (sine amplitude = sqrt(2*power))
%   R.theta_ref    reference angle [rad]
if nargin < 2, show = true; end
Ts = 1/L.fs; N = numel(L.t);

% complementary filter
tau = 0.5; a = tau/(tau + Ts);
th = zeros(N,1); th(1) = L.theta_mes(1);
for k = 2:N
    th(k) = a*(th(k-1) + Ts*L.omega_mes(k)) + (1 - a)*L.theta_mes(k);
end
R.theta_ref = th;
x = th*180/pi;

% Welch
nw = round(2/Ts); hop = round(nw/2);
w = 0.5 - 0.5*cos(2*pi*(0:nw-1)'/nw);       % Hann
U = sum(w.^2);
starts = 1:hop:N-nw+1;
P = zeros(floor(nw/2)+1, 1);
for s = starts
    seg = x(s:s+nw-1); seg = (seg - mean(seg)).*w;
    X = fft(seg);
    P = P + abs(X(1:floor(nw/2)+1)).^2;
end
P = P/(numel(starts)*U*L.fs);
P(2:end-1) = 2*P(2:end-1);                  % one-sided
f = (0:floor(nw/2))'*L.fs/nw;
R.f = f; R.P = P;

band = f >= 3 & f <= 8;
[~, i] = max(P.*band);
R.f_peak = f(i);
near = abs(f - R.f_peak) <= 0.5;
R.amp_peak = sqrt(2*sum(P(near))*(f(2) - f(1)));

if show
    % spectrogram: 2 s windows every 0.25 s
    hop2 = round(0.25/Ts); st2 = 1:hop2:N-nw+1;
    S = zeros(floor(nw/2)+1, numel(st2));
    for j = 1:numel(st2)
        seg = x(st2(j):st2(j)+nw-1); seg = (seg - mean(seg)).*w;
        X = fft(seg); S(:,j) = abs(X(1:floor(nw/2)+1)).^2/(U*L.fs);
    end
    tc = L.t(st2 + round(nw/2));
    figure('Name', ['PSD ' L.name], 'Color', 'w');
    subplot(3,1,1); plot(L.t, x, 'k'); grid on;
    ylabel('\theta [deg]'); title(strrep(L.name, '_', '\_'));
    subplot(3,1,2); plot(f, 10*log10(P + eps), 'k', 'LineWidth', 1.2); grid on;
    xlim([0 15]); ylim([-60 30]); ylabel('PSD [dB deg^2/Hz]');
    hold on; plot(R.f_peak, 10*log10(P(i)), 'ro');
    text(R.f_peak + 0.3, 10*log10(P(i)), sprintf('%.2f Hz, %.1f deg', R.f_peak, R.amp_peak));
    subplot(3,1,3); imagesc(tc, f, 10*log10(S + eps)); axis xy;
    ylim([0 15]); caxis([-60 30]); xlabel('time [s]'); ylabel('f [Hz]'); colorbar;
end
end

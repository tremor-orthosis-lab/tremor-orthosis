function id = identify(path, show)
%IDENTIFY Wrist + orthosis parameters from an IDENT log (task J-3.3/J-3.4).
%   id = identify('../data/raw/20261102_101500_ident.csv')
%
%   Recording: hand relaxed, motors ARMed, firmware command
%       LOG on   then   IDENT 0.05 0.5 12 30
%   (torque chirp 0.05 N m from 0.5 to 12 Hz over 30 s).
%
%   Method (frequency domain, no toolbox):
%   1. Transfer function from the commanded torque to the wrist rate
%      (gyro gy1 - gy2, much cleaner than the accelerometer angle):
%        H(f) = S_tau,omega(f) / S_tau,tau(f)    (Welch, 4 s windows)
%      The external torques of the person are not correlated with the
%      chirp, so they average out.
%   2. Fit of the model  omega/tau = jw e^(-jw d) / (K - J w^2 + jB w)
%      on the frequencies where the coherence is > 0.8, by fminsearch on
%      the log of the complex ratio (magnitude and phase errors together).
%   Result: J, K, B and the total delay d (sensor filter + loop + driver).
%   Note: J, K, B are identified relative to the COMMANDED torque, so an
%   error in the motor torque constant is absorbed in them (what the
%   controller needs).
%
%   id.J, id.K, id.B, id.delay_s, id.f_n (natural frequency), id.fit_err
%   Then: copy them into tremor_params.m (p.plant) and re-run
%   design_controller.m to re-tune the gains on the real wrist.
if nargin < 2, show = true; end
L = load_log(path);
Ts = 1/L.fs;
u = L.tau_cmd;
y = L.omega_mes;
if max(abs(u)) == 0
    error('identify: tau_cmd is zero in this log. Was IDENT running (and the log on)?');
end

% --- 1. empirical transfer function
nw = round(4/Ts); hop = round(nw/2);
w = 0.5 - 0.5*cos(2*pi*(0:nw-1)'/nw);
Suu = 0; Suy = 0; Syy = 0;
for s = 1:hop:numel(u)-nw+1
    U = fft(w.*(u(s:s+nw-1) - mean(u(s:s+nw-1))));
    Y = fft(w.*(y(s:s+nw-1) - mean(y(s:s+nw-1))));
    Suu = Suu + abs(U).^2; Suy = Suy + conj(U).*Y; Syy = Syy + abs(Y).^2;
end
f = (0:nw-1)'*L.fs/nw;
H = Suy./Suu;
coh = abs(Suy).^2./(Suu.*Syy);
use = f >= 0.7 & f <= 11 & coh > 0.8;
if nnz(use) < 10
    error('identify: too few usable frequencies (coherence). Larger IDENT amplitude?');
end
fu = f(use); Hu = H(use); wu = 2*pi*fu;

% --- 2. fit, starting from the nominal values of tremor_params.m (C4)
p = tremor_params();
x0 = [log(p.plant.J), log(p.plant.K), log(p.plant.B), p.plant.delay_steps*p.Ts];
model = @(x, ww) 1i*ww.*exp(-1i*ww*x(4)) ./ (exp(x(2)) - exp(x(1))*ww.^2 + 1i*exp(x(3))*ww);
cost  = @(x) sum(abs(log(Hu ./ model(x, wu))).^2);
opts = optimset('MaxFunEvals', 4000, 'MaxIter', 4000, 'TolX', 1e-8, 'TolFun', 1e-10);
x = fminsearch(cost, x0, opts);
x = fminsearch(cost, x, opts);          % restart once (Nelder-Mead)

id.J = exp(x(1)); id.K = exp(x(2)); id.B = exp(x(3)); id.delay_s = x(4);
id.f_n = sqrt(id.K/id.J)/(2*pi);
id.fit_err = sqrt(cost(x)/numel(wu));   % RMS of |log ratio| (0.1 ~ 10 %)
id.n_freq = numel(wu);

fprintf('\n=== Identification: %s ===\n', L.name);
fprintf('  %d frequencies used (%.1f-%.1f Hz, coherence > 0.8)\n', id.n_freq, min(fu), max(fu));
fprintf('  J = %.2e kg m^2   (nominal %.2e)\n', id.J, p.plant.J);
fprintf('  K = %.3f N m/rad  (nominal %.3f)\n', id.K, p.plant.K);
fprintf('  B = %.4f N m s/rad (nominal %.4f)\n', id.B, p.plant.B);
fprintf('  delay = %.1f ms   (nominal %.1f ms)\n', 1e3*id.delay_s, 1e3*p.plant.delay_steps*p.Ts);
fprintf('  natural frequency %.2f Hz, fit error %.2f\n', id.f_n, id.fit_err);
fprintf('Next: put these values in tremor_params.m (p.plant, delay_steps = %d) and re-run design_controller.\n', ...
        max(1, round(id.delay_s/p.Ts)));

if show
    G = model(x, 2*pi*f);
    k = f > 0.3 & f < 13;
    figure('Name', ['Identification ' L.name], 'Color', 'w');
    subplot(3,1,1);
    semilogx(f(k), 20*log10(abs(H(k))), 'k.', f(k), 20*log10(abs(G(k))), 'r', 'LineWidth', 1.2);
    grid on; ylabel('|\omega/\tau| [dB]'); legend('measured', 'fitted model', 'Location', 'best');
    title(strrep(L.name, '_', '\_'));
    subplot(3,1,2);
    semilogx(f(k), unwrap(angle(H(k)))*180/pi, 'k.', f(k), unwrap(angle(G(k)))*180/pi, 'r', 'LineWidth', 1.2);
    grid on; ylabel('phase [deg]');
    subplot(3,1,3);
    semilogx(f(k), coh(k), 'k'); hold on; yline(0.8, 'r:'); grid on;
    ylim([0 1.05]); xlabel('f [Hz]'); ylabel('coherence');
end
end

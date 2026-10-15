function [f_hat, active, u] = wflc_freq(y_omega, p)
%WFLC_FREQ Online estimate of the tremor frequency (task J-4.1).
%   [f_hat, active, u] = wflc_freq(y_omega, p)
%     y_omega : N x 1 relative gyro rate [rad/s] (2nd column of y)
%     p       : parameters from tremor_params() (fields p.Ts and p.wflc)
%   f_hat  : N x 1 estimated tremor frequency [Hz], to be passed to
%            tremor_kalman(y, f_hat, p)
%   active : N x 1 true where the frequency was adapting (tremor present)
%   u      : N x 1 band-passed input (for plots / debugging)
%
%   Method: Weighted-frequency Fourier Linear Combiner (Riviere, Rader &
%   Thakor, IEEE TBME 1998), one harmonic. The WFLC fits
%       u(k) ~ w1*sin(phi(k)) + w2*cos(phi(k)),  phi(k+1) = phi(k) + om(k)
%   and adapts the weights w (LMS) and the frequency om by gradient descent
%   on the fitting error e:
%       om <- om + 2*mu0*e*(w1*cos(phi) - w2*sin(phi))
%       w  <- w  + 2*mu1*e*[sin(phi); cos(phi)]
%
%   Three additions to the textbook WFLC:
%     1. Band-pass 3-10 Hz on the gyro (removes most voluntary motion).
%        The gyro is used rather than the angle: differentiation boosts
%        the tremor relative to the slow voluntary motion.
%     2. AGC: the input is divided by its running RMS, so mu0 gives the
%        same adaptation speed for a 1 deg or a 5 deg tremor.
%     3. Gate: if the band RMS is below p.wflc.gate (no tremor), the
%        frequency is frozen. Without it the estimate drifts towards 3 Hz
%        during pure voluntary motion and leakage triples.
%
%   Sample-by-sample, no toolbox, ~20 flops + 1 sin + 1 cos per sample:
%   ported as is to the ESP32.

w  = p.wflc;
Ts = p.Ts;
N  = numel(y_omega);

% Band-pass coefficients (same RBJ design as bandpass_biquad.m)
fc = sqrt(w.f_lo*w.f_hi);
Qf = fc/(w.f_hi - w.f_lo);
wc = 2*pi*fc*Ts;
al = sin(wc)/(2*Qf);
b  = [al, 0, -al]/(1 + al);
a  = [1, -2*cos(wc)/(1 + al), (1 - al)/(1 + al)];

lam   = Ts/w.tau_agc;               % AGC forgetting factor
om_lo = 2*pi*w.f_min*Ts;            % frequency clamp [rad/sample]
om_hi = 2*pi*w.f_max*Ts;

f_hat  = zeros(N,1);
active = false(N,1);
u      = zeros(N,1);

x1 = 0; x2 = 0; u1 = 0; u2 = 0;     % biquad memory
ms = 0;                             % running mean square of u
om = 2*pi*w.f_init*Ts;              % frequency [rad/sample]
ph = 0;                             % phase of the reference oscillator
wt = [0; 0];                        % Fourier weights

for k = 1:N
    % 1. band-pass (direct form I)
    xk = y_omega(k);
    uk = b(1)*xk + b(2)*x1 + b(3)*x2 - a(2)*u1 - a(3)*u2;
    x2 = x1; x1 = xk; u2 = u1; u1 = uk;

    % 2. amplitude normalisation
    ms  = (1 - lam)*ms + lam*uk^2;
    rms = sqrt(ms);
    sk  = uk / max(rms, w.gate);

    % 3. WFLC
    ph = mod(ph + om, 2*pi);
    xr = [sin(ph); cos(ph)];
    e  = sk - wt'*xr;
    if rms > w.gate                         % tremor present: adapt f
        om = om + 2*w.mu0*e*(wt(1)*xr(2) - wt(2)*xr(1));
        om = min(max(om, om_lo), om_hi);
        active(k) = true;
    end
    wt = wt + 2*w.mu1*e*xr;

    f_hat(k) = om/(2*pi*Ts);
    u(k) = uk;
end
end

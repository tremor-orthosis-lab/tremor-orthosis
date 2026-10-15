function yf = bandpass_biquad(x, f_lo, f_hi, Ts)
%BANDPASS_BIQUAD Causal 2nd-order band-pass (RBJ cookbook, 0 dB peak).
%   yf = bandpass_biquad(x, f_lo, f_hi, Ts)
%   Baseline estimator for comparison with the Kalman filter: centre
%   frequency sqrt(f_lo*f_hi), bandwidth f_hi - f_lo. No toolbox needed.
fs = 1/Ts;
f0 = sqrt(f_lo*f_hi);
Qf = f0/(f_hi - f_lo);
w0 = 2*pi*f0/fs;
alpha = sin(w0)/(2*Qf);
b = [alpha, 0, -alpha];
a = [1 + alpha, -2*cos(w0), 1 - alpha];
b = b/a(1); a = a/a(1);
yf = filter(b, a, x);
end

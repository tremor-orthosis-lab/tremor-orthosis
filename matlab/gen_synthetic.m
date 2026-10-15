function s = gen_synthetic(f_trem, T, Ts, seed, amp_deg)
%GEN_SYNTHETIC Synthetic wrist motion = voluntary + tremor + sensor noise.
%   s = gen_synthetic(f_trem, T, Ts, seed, amp_deg)
%     f_trem : tremor frequency [Hz], either a scalar (e.g. 4, 5 or 6; use
%              0 for no tremor) or a function handle f(t) for a frequency
%              that changes over time, e.g. @(t) 4 + 2*t/T (chirp)
%     T      : duration [s]
%     Ts     : sample time [s]
%     seed   : random seed (for reproducible tests)
%     amp_deg: mean tremor amplitude [deg] (default 5)
%   Extra field s.f_true : true tremor frequency at each sample [Hz]
%   Fields of s (all column vectors, rad and rad/s):
%     t, theta_v, omega_v, theta_t, omega_t   ground truth
%     y = [theta_mes, omega_mes]               noisy measurements
%
%   Model of the measurements (same as tremor_kalman.m):
%     theta_mes = theta_v + theta_t + noise (accelerometer tilt, std 2 deg)
%     omega_mes = omega_v + omega_t + noise (gyro2 - gyro1, std 0.05 rad/s)
%   Limits of this synthetic test: white noise only, no gyro bias, no
%   corruption of the tilt by the hand acceleration. Real logs (J-1.3) will
%   be harder.

if nargin < 4, seed = 1; end
if nargin < 5, amp_deg = 5; end
rng(seed);
d = pi/180;
t = (0:Ts:T-Ts)';

% Voluntary motion: slow sum of sines between 0.2 and 1.3 Hz (~20 deg pk)
fv = [0.2 0.45 0.8 1.3];
av = [12 6 3 1.5]*d;
ph = 2*pi*rand(1,numel(fv));
theta_v = zeros(size(t)); omega_v = zeros(size(t));
for i = 1:numel(fv)
    w = 2*pi*fv(i);
    theta_v = theta_v + av(i)*sin(w*t + ph(i));
    omega_v = omega_v + av(i)*w*cos(w*t + ph(i));
end

% Tremor: amp_deg amplitude, slowly modulated (+/-30 % at 0.1 Hz)
amp  = amp_deg*d*(1 + 0.3*sin(2*pi*0.1*t));
damp = amp_deg*d*0.3*2*pi*0.1*cos(2*pi*0.1*t);
if isa(f_trem, 'function_handle')
    % time-varying frequency: integrate it to get the phase
    f_true = f_trem(t);
    f_true = f_true(:) .* ones(size(t));
    phase  = 2*pi*cumsum(f_true)*Ts;
    theta_t = amp.*sin(phase);
    omega_t = damp.*sin(phase) + amp.*(2*pi*f_true).*cos(phase);
elseif f_trem > 0
    wt  = 2*pi*f_trem;
    f_true = f_trem*ones(size(t));
    theta_t = amp.*sin(wt*t);
    omega_t = damp.*sin(wt*t) + amp.*wt.*cos(wt*t);
else
    f_true = zeros(size(t));
    theta_t = zeros(size(t)); omega_t = zeros(size(t));
end

p = tremor_params();
y = [theta_v + theta_t + p.sigma_theta*randn(size(t)), ...
     omega_v + omega_t + p.sigma_omega*randn(size(t))];

s = struct('t',t,'theta_v',theta_v,'omega_v',omega_v, ...
           'theta_t',theta_t,'omega_t',omega_t,'y',y,'f_true',f_true);
s.f_trem = f_trem;   % set after struct() so a handle is stored as is
end

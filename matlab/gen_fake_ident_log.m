function truth = gen_fake_ident_log(path, pl, delay_s, seed)
%GEN_FAKE_IDENT_LOG Synthetic identification log, to test identify.m.
%   truth = gen_fake_ident_log(path, pl, delay_s, seed)
%   Simulates the firmware command "IDENT 0.05 0.5 12 30" (same chirp as
%   src/control/controller.cpp: linear 0.5 -> 12 Hz, 0.5 s fade) on a wrist
%   with parameters pl (J, K, B) and a pure delay delay_s on the torque,
%   plus a small random external torque (a relaxed hand is never perfectly
%   passive). IMU readings from imu_from_motion.m.
%   truth = the parameters used, to compare with identify.m.
if nargin < 2 || isempty(pl), pl = struct('J', 2.0e-3, 'K', 0.45, 'B', 0.03); end
if nargin < 3, delay_s = 0.008; end
if nargin < 4, seed = 1; end
Ts = 0.002; T = 30; A = 0.05; f1 = 0.5; f2 = 12; fade = 0.5;
t = (0:Ts:T-Ts)'; N = numel(t);
f = f1 + (f2 - f1)*t/T;
ph = cumsum(2*pi*f*Ts);
env = min(1, min(t/fade, (T - t)/fade));
tau_cmd = A*env.*sin(ph);
% small external torque: low-pass noise, ~0.005 N m RMS
rng(seed);
dist = filter(0.01, [1 -0.99], randn(N,1)); dist = 0.005*dist/std(dist);
nd = round(delay_s/Ts);
u = [zeros(nd,1); tau_cmd(1:end-nd)] + dist;
pm = plant_model(pl, Ts);
x = [0; 0]; th = zeros(N,1);
for k = 1:N
    th(k) = x(1);
    x = pm.Ad*x + pm.Bd*u(k);
end
[acc1, gyr1, acc2, gyr2] = imu_from_motion(th, Ts, seed);
write_log(path, t, acc1, gyr1, acc2, gyr2, tau_cmd);
truth = pl; truth.delay_s = delay_s;
end

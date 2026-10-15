function gen_fake_log(path, f_trem, T, amp_deg, seed)
%GEN_FAKE_LOG Write a synthetic log in the firmware CSV format (C3).
%   gen_fake_log(path, f_trem, T, amp_deg, seed)
%   Used to test load_log / psd_analysis / analyze_log before the hardware
%   arrives, and later the metrics pipeline (J-4.2).
%   Wrist motion = gen_synthetic (voluntary + tremor). The two IMUs are
%   simulated from the kinematics, with the frames and signs of C2:
%     forearm IMU1 still, tilted 3 deg; hand IMU2 at alpha2 = alpha1 - theta
%     gyro y: g1y = 0, g2y = -dtheta/dt  (so g1y - g2y = +dtheta/dt)
%     accelerometers: gravity + the acceleration of the hand IMU, mounted
%     7 cm beyond the wrist axis and 2 cm above it (r = [0.07 0 0.02] m):
%       a = dw/dt x r + w x (w x r),  w = [0; w2; 0]
%     This is the effect that corrupts the accelerometer angle during tremor.
%   Sensor noise: accelerometer 0.04 m/s^2, gyro 0.002 rad/s (MPU-6050 at
%   98 Hz bandwidth). Estimator columns are written as 0.
if nargin < 5, seed = 1; end
Ts = 0.002; g = 9.80665;
s = gen_synthetic(f_trem, T, Ts, seed, amp_deg);
rng(seed + 100);
N = numel(s.t);
th = s.theta_v + s.theta_t;
w2 = -(s.omega_v + s.omega_t);            % hand rate about y
dw2 = gradient(w2, Ts);
a1 = 3*pi/180*ones(N,1);
a2 = a1 - th;
r = [0.07 0 0.02];
acc_kin = [dw2*r(3) - w2.^2*r(1), zeros(N,1), -dw2*r(1) - w2.^2*r(3)];
acc1 = [-g*sin(a1), zeros(N,1), g*cos(a1)] + 0.04*randn(N,3);
acc2 = [-g*sin(a2), zeros(N,1), g*cos(a2)] + acc_kin + 0.04*randn(N,3);
gyr1 = 0.002*randn(N,3);
gyr2 = [zeros(N,1), w2, zeros(N,1)] + 0.002*randn(N,3);
n = (1:N)';
t_us = round(1e6 + s.t*1e6);
D = [n, t_us, acc1, gyr1, acc2, gyr2, zeros(N,6), 400*ones(N,1)];
fid = fopen(path, 'w');
fprintf(fid, 'n,t_us,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,theta_hat,theta_v_hat,theta_t_hat,f_hat,tau_cmd,vbat,loop_us\n');
fprintf(fid, ['%d,%d,' repmat('%.4f,', 1, 18) '%d\n'], D');
fclose(fid);
end

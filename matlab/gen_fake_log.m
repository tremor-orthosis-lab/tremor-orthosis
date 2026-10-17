function gen_fake_log(path, f_trem, T, amp_deg, seed)
%GEN_FAKE_LOG Write a synthetic log in the firmware CSV format (C3).
%   gen_fake_log(path, f_trem, T, amp_deg, seed)
%   Used to test load_log / psd_analysis / analyze_log before the hardware
%   arrives, and later the metrics pipeline (J-4.2).
%   Wrist motion = gen_synthetic (voluntary + tremor); IMU readings from
%   imu_from_motion.m (C2 frames, hand acceleration seen by the hand
%   accelerometer, MPU-6050 noise).
if nargin < 5, seed = 1; end
Ts = 0.002;
s = gen_synthetic(f_trem, T, Ts, seed, amp_deg);
[acc1, gyr1, acc2, gyr2] = imu_from_motion(s.theta_v + s.theta_t, Ts, seed);
write_log(path, s.t, acc1, gyr1, acc2, gyr2);
end

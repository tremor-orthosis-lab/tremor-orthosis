function [acc1, gyr1, acc2, gyr2] = imu_from_motion(th, Ts, seed)
%IMU_FROM_MOTION Simulated IMU readings for a wrist angle th(t) (C2 frames).
%   [acc1, gyr1, acc2, gyr2] = imu_from_motion(th, Ts, seed), N x 3 each.
%   Forearm IMU1 still, tilted 3 deg; hand IMU2 at alpha2 = alpha1 - theta
%   gyro y: g1y = 0, g2y = -dtheta/dt  (so g1y - g2y = +dtheta/dt)
%   accelerometers: gravity + acceleration of the hand IMU, mounted 7 cm
%   beyond the wrist axis and 2 cm above it (r = [0.07 0 0.02] m):
%     a = dw/dt x r + w x (w x r),  w = [0; w2; 0]
%   Noise: accelerometer 0.04 m/s^2, gyro 0.002 rad/s (MPU-6050, 98 Hz).
if nargin < 3, seed = 1; end
rng(seed + 100);
g = 9.80665; th = th(:); N = numel(th);
w2  = -gradient(th, Ts);
dw2 = gradient(w2, Ts);
a1 = 3*pi/180*ones(N,1);
a2 = a1 - th;
r = [0.07 0 0.02];
acc_kin = [dw2*r(3) - w2.^2*r(1), zeros(N,1), -dw2*r(1) - w2.^2*r(3)];
acc1 = [-g*sin(a1), zeros(N,1), g*cos(a1)] + 0.04*randn(N,3);
acc2 = [-g*sin(a2), zeros(N,1), g*cos(a2)] + acc_kin + 0.04*randn(N,3);
gyr1 = 0.002*randn(N,3);
gyr2 = [zeros(N,1), w2, zeros(N,1)] + 0.002*randn(N,3);
end

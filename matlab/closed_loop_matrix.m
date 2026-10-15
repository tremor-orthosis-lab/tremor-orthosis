function [M, Bd] = closed_loop_matrix(f0, kgain, p, pl, nd)
%CLOSED_LOOP_MATRIX Linear model of plant + Kalman + delay + controller.
%   [M, Bd] = closed_loop_matrix(f0, kgain, p, pl, nd)
%     f0    : model frequency of the Kalman filter [Hz] (fixed)
%     kgain : 1x4 gain, tau = -kgain * x_hat, x_hat = [th_v om_v th_t om_t]
%     pl    : plant parameters (J, K, B), nd : delay in samples (>= 1)
%   State z = [theta; omega; x_hat (4); u(k-1) ... u(k-nd)]
%   z(k+1) = M z(k) + Bd * d(k), d = external torque (tremor or voluntary)
%   Noise is ignored (linear analysis). Stable iff max(abs(eig(M))) < 1.
%   Response to a torque at f [Hz]: first entry of (exp(1i*2*pi*f*Ts)*I - M) \ Bd.
pm = plant_model(pl, p.Ts);
[A, L] = kalman_steady(f0, p);
H = [1 0 1 0; 0 1 0 1];
n = 2 + 4 + nd;
M = zeros(n); Bd = zeros(n,1);
M(1:2,1:2) = pm.Ad;                 % plant, driven by the delayed command
M(1:2,6+nd) = pm.Bd;
Bd(1:2) = pm.Bd;
M(3:6,3:6) = (eye(4) - L*H)*A;      % Kalman: predict, then update with
M(3:6,:) = M(3:6,:) + L*M(1:2,:);   % the new measurement y = [theta; omega]
Bd(3:6) = L*Bd(1:2);
M(7,3:6) = -kgain;                  % u(k) = -K x_hat(k), then shift
for i = 1:nd-1
    M(7+i,6+i) = 1;
end
end

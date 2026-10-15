function [A, L] = kalman_steady(f0, p)
%KALMAN_STEADY Steady-state gain of tremor_kalman.m for a fixed f0.
%   [A, L] = kalman_steady(f0, p): A = state matrix, L = 4x2 Kalman gain.
%   Used for the linear closed-loop analysis (closed_loop_matrix.m). After
%   ~1 s the gain of the time-varying filter equals L.
[~, info] = tremor_kalman(zeros(1,2), f0, p);
A = info.A; Q = info.Q; H = info.H; R = info.R;
P = p.P0;
for k = 1:4000
    Pp = A*P*A' + Q;
    L  = (Pp*H') / (H*Pp*H' + R);
    P  = (eye(4) - L*H)*Pp;
end
end

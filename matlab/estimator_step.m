function [st, xh, f_hat] = estimator_step(st, y, p)
%ESTIMATOR_STEP One sample of WFLC + Kalman, for closed-loop simulation.
%   st = estimator_step([], [], p) initialises the state.
%   [st, xh, f_hat] = estimator_step(st, [theta_mes omega_mes], p)
%   Same equations as wflc_freq.m + tremor_kalman.m (which process a whole
%   recording at once and are the reference for the C++ port), rewritten
%   sample by sample because in closed loop the next measurement depends
%   on the torque computed from this estimate.
if isempty(st)
    w = p.wflc; Ts = p.Ts;
    fc = sqrt(w.f_lo*w.f_hi); Qf = fc/(w.f_hi - w.f_lo);
    wc = 2*pi*fc*Ts; al = sin(wc)/(2*Qf);
    st.b = [al, 0, -al]/(1 + al);
    st.a = [1, -2*cos(wc)/(1 + al), (1 - al)/(1 + al)];
    st.z = [0 0 0 0];                 % x1 x2 u1 u2 of the biquad
    st.ms = 0; st.om = 2*pi*w.f_init*Ts; st.ph = 0; st.wt = [0; 0];
    G = [Ts^3/3, Ts^2/2; Ts^2/2, Ts];
    st.Q = blkdiag(p.q_v*G, p.q_t*G);
    st.H = [1 0 1 0; 0 1 0 1];
    st.R = diag([p.r_theta_scale*p.sigma_theta^2, p.sigma_omega^2]);
    st.x = p.x0; st.P = p.P0; st.first = true;
    xh = st.x; f_hat = w.f_init;
    return
end
w = p.wflc; Ts = p.Ts;
if st.first, st.x(1) = y(1); st.first = false; end   % seed theta_v
% --- WFLC on the gyro rate
z = st.z; xk = y(2);
uk = st.b(1)*xk + st.b(2)*z(1) + st.b(3)*z(2) - st.a(2)*z(3) - st.a(3)*z(4);
st.z = [xk, z(1), uk, z(3)];
st.ms = (1 - Ts/w.tau_agc)*st.ms + Ts/w.tau_agc*uk^2;
rms = sqrt(st.ms);
sk = uk / max(rms, w.gate);
st.ph = mod(st.ph + st.om, 2*pi);
xr = [sin(st.ph); cos(st.ph)];
e = sk - st.wt'*xr;
if rms > w.gate
    st.om = st.om + 2*w.mu0*e*(st.wt(1)*xr(2) - st.wt(2)*xr(1));
    st.om = min(max(st.om, 2*pi*w.f_min*Ts), 2*pi*w.f_max*Ts);
end
st.wt = st.wt + 2*w.mu1*e*xr;
f_hat = st.om/(2*pi*Ts);
% --- Kalman with the model at f_hat
w0 = 2*pi*f_hat; c = cos(w0*Ts); s = sin(w0*Ts);
A = [1 Ts 0 0; 0 1 0 0; 0 0 p.rho*c p.rho*s/w0; 0 0 -p.rho*w0*s p.rho*c];
x = A*st.x; P = A*st.P*A' + st.Q;
K = (P*st.H') / (st.H*P*st.H' + st.R);
st.x = x + K*(y(:) - st.H*x);
P = (eye(4) - K*st.H)*P;
st.P = (P + P')/2;
xh = st.x;
end

function [X, info] = tremor_kalman(y, f0, p)
%TREMOR_KALMAN Discrete Kalman filter separating voluntary motion and tremor.
%   [X, info] = tremor_kalman(y, f0, p)
%     y  : N x 2 measurements [theta_mes, omega_mes] (rad, rad/s)
%     f0 : tremor frequency used by the model [Hz], either
%          - a scalar  : fixed model frequency (J-1.5), or
%          - an N x 1 vector : model frequency updated at every sample,
%            e.g. the output of wflc_freq.m (adaptive filter, J-4.1)
%     p  : parameters from tremor_params()
%   X : N x 4 filtered states  [theta_v, omega_v, theta_t, omega_t]
%   info.A, info.Q, info.H, info.R : matrices used (for the C++ port;
%   with a frequency vector, info.A is the matrix of the last sample)
%
%   State model (sample time Ts):
%     voluntary : constant velocity driven by white acceleration
%                 [theta_v; omega_v](k+1) = [1 Ts; 0 1] [theta_v; omega_v](k)
%     tremor    : damped discrete harmonic oscillator at w0 = 2*pi*f0
%                 [theta_t; omega_t](k+1) = rho*[ c  s/w0 ; -w0*s  c ] [..](k)
%                 with c = cos(w0*Ts), s = sin(w0*Ts)
%     measure   : y = H x + v,  H = [1 0 1 0 ; 0 1 0 1]
%
%   Written with plain matrix operations (no toolbox) so that it can be
%   ported line by line to C++ on the ESP32 (task J-2.4). In the adaptive
%   case only the 2x2 tremor block of A changes: one cos and one sin per
%   sample on the ESP32.

Ts = p.Ts;
N = size(y,1);
adaptive = numel(f0) > 1;
if adaptive && numel(f0) ~= N
    error('tremor_kalman: f0 must be a scalar or have one value per sample');
end

A = build_A(f0(1), p);

G = [Ts^3/3, Ts^2/2; Ts^2/2, Ts];      % white-acceleration discretisation
Q = blkdiag(p.q_v*G, p.q_t*G);

H = [1 0 1 0; 0 1 0 1];
R = diag([p.r_theta_scale*p.sigma_theta^2, p.sigma_omega^2]);

X = zeros(N,4);
x = p.x0; P = p.P0; I4 = eye(4);
for k = 1:N
    if adaptive
        A = build_A(f0(k), p);           % model follows the estimated frequency
    end
    % Predict
    x = A*x;
    P = A*P*A' + Q;
    % Update
    S = H*P*H' + R;
    K = (P*H') / S;
    x = x + K*(y(k,:)' - H*x);
    P = (I4 - K*H)*P;
    P = (P + P')/2;                      % keep P symmetric
    X(k,:) = x';
end

info = struct('A',A,'Q',Q,'H',H,'R',R,'K_last',K,'P_last',P);
end

function A = build_A(f, p)
% State transition matrix for a model tremor frequency f [Hz].
Ts = p.Ts;
w0 = 2*pi*f;
c = cos(w0*Ts); s = sin(w0*Ts);
A = zeros(4);
A(1:2,1:2) = [1 Ts; 0 1];
A(3:4,3:4) = p.rho*[c, s/w0; -w0*s, c];
end

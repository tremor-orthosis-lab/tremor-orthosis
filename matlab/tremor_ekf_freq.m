function [X, f_hat] = tremor_ekf_freq(y, p)
%TREMOR_EKF_FREQ Extended Kalman filter with the tremor frequency as a state.
%   [X, f_hat] = tremor_ekf_freq(y, p)
%     y : N x 2 measurements [theta_mes, omega_mes] (rad, rad/s)
%     p : parameters from tremor_params() (uses p.ekf for the frequency)
%   X     : N x 5 states [theta_v, omega_v, theta_t, omega_t, w] (w in rad/s)
%   f_hat : N x 1 estimated frequency [Hz]
%
%   Alternative to wflc_freq + tremor_kalman, kept for comparison (J-4.1).
%   Same model as tremor_kalman.m, plus w(k+1) = w(k) + random walk.
%   The tremor block depends on w, so the model is nonlinear and the
%   covariance is propagated with the Jacobian F = df/dx:
%     d/dw of rho*[ c*th + s/w*om ]  with c = cos(w Ts), s = sin(w Ts)
%               [ -w*s*th + c*om ]
%     = rho*[ -Ts*s*th + (Ts*c*w - s)/w^2*om ;
%             -(s + w*Ts*c)*th - Ts*s*om      ]
%   Cost on the ESP32: 5x5 instead of 4x4 matrices, and no gate (it
%   adapts on noise when there is no tremor).

Ts = p.Ts; e = p.ekf; rho = p.rho;
N  = size(y,1);

G  = [Ts^3/3, Ts^2/2; Ts^2/2, Ts];
Q  = blkdiag(p.q_v*G, p.q_t*G, e.q_w*Ts);
H  = [1 0 1 0 0; 0 1 0 1 0];
R  = diag([p.r_theta_scale*p.sigma_theta^2, p.sigma_omega^2]);
wlo = 2*pi*e.f_min; whi = 2*pi*e.f_max;

x = [p.x0; 2*pi*e.f_init];
P = blkdiag(p.P0, e.p_w0);
I5 = eye(5);
X = zeros(N,5);
for k = 1:N
    w = x(5); c = cos(w*Ts); s = sin(w*Ts);
    th = x(3); om = x(4);

    % Jacobian (evaluated at the previous estimate)
    F = I5;
    F(1,2) = Ts;
    F(3:4,3:4) = rho*[c, s/w; -w*s, c];
    F(3,5) = rho*(-Ts*s*th + (Ts*c*w - s)/w^2*om);
    F(4,5) = rho*(-(s + w*Ts*c)*th - Ts*s*om);

    % Predict (nonlinear model)
    x(1) = x(1) + Ts*x(2);
    x(3) = rho*( c*th + s/w*om);
    x(4) = rho*(-w*s*th + c*om);
    P = F*P*F' + Q;

    % Update (linear measurement)
    S = H*P*H' + R;
    K = (P*H') / S;
    x = x + K*(y(k,:)' - H*x);
    P = (I5 - K*H)*P;
    P = (P + P')/2;
    x(5) = min(max(x(5), wlo), whi);
    X(k,:) = x';
end
f_hat = X(:,5)/(2*pi);
end

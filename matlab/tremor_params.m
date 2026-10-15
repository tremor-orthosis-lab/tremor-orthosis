function p = tremor_params()
%TREMOR_PARAMS Default parameters of the tremor/voluntary Kalman filter.
%   p = tremor_params() returns a struct used by tremor_kalman.m and by the
%   synthetic test. Units: rad, rad/s, s. Values tuned on synthetic data
%   (run_kalman_synthetic.m); they must be re-tuned on real IMU logs (J-1.3).

p.Ts   = 0.002;        % sample time [s] (500 Hz loop, contract C4)

% --- Process noise (continuous white acceleration intensities) ---------
% Large q_v: the voluntary state is allowed to change quickly, so slow
% motion is absorbed by the voluntary states and does not leak into tremor.
% Small q_t: the tremor states follow the oscillator model closely.
p.q_v  = 200;          % voluntary  [rad^2/s^3]
p.q_t  = 1000;         % tremor     [rad^2/s^3]
p.rho  = 0.9998;       % oscillator damping per step (1 = undamped)

% --- Measurement noise ---------------------------------------------------
% theta_mes comes from accelerometer tilt: noisy AND corrupted by the hand
% acceleration during tremor -> trusted much less than the gyro.
p.sigma_theta = 2*pi/180;  % [rad] std of theta measurement (synthetic)
p.r_theta_scale = 25;      % inflate R_theta (accel tilt is unreliable)
p.sigma_omega = 0.05;      % [rad/s] std of relative gyro rate

% --- Initial state and covariance ----------------------------------------
p.x0 = zeros(4,1);
p.P0 = 1e-2*eye(4);

% --- Adaptive frequency estimator: WFLC (wflc_freq.m, task J-4.1) --------
% Input: relative gyro rate, band-passed and normalised (AGC) so that the
% adaptation speed does not depend on the tremor amplitude.
p.wflc.f_lo   = 3;      % band-pass of the input [Hz]
p.wflc.f_hi   = 10;
p.wflc.tau_agc = 1.0;   % time constant of the amplitude normalisation [s]
p.wflc.gate   = 0.25;   % [rad/s] band RMS below which f is frozen
                        % (~0.6 deg of tremor at 5 Hz; voluntary motion
                        %  alone gives ~0.1 rad/s in this band)
p.wflc.mu0    = 2e-4;   % frequency adaptation gain
p.wflc.mu1    = 5e-3;   % amplitude-weight adaptation gain
p.wflc.f_init = 5;      % [Hz] initial guess
p.wflc.f_min  = 3;      % [Hz] clamp of the estimate
p.wflc.f_max  = 8;

% --- Alternative: extended Kalman filter with the frequency as 5th state --
p.ekf.q_w    = 2*(2*pi)^2;  % random-walk intensity of w [rad^2/s^3]
p.ekf.p_w0   = (2*pi*1)^2;  % initial variance of w (1 Hz std)
p.ekf.f_init = 5;
p.ekf.f_min  = 3;
p.ekf.f_max  = 8;
end

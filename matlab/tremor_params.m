function p = tremor_params()
%TREMOR_PARAMS Default parameters of the tremor/voluntary Kalman filter.
%   p = tremor_params() returns a struct used by tremor_kalman.m and by the
%   synthetic test. Units: rad, rad/s, s. Values tuned on synthetic data
%   (run_kalman_synthetic.m); they must be re-tuned on real IMU logs (J-1.3).

p.Ts   = 0.002;        % sample time [s] (500 Hz loop, contract C4)

% --- Process noise (continuous white acceleration intensities) ---------
% Large q_v: the voluntary state is allowed to change quickly, so slow
% motion is absorbed by the voluntary states and does not leak into tremor.
% Smaller q_t: the tremor states follow the oscillator model closely.
% Re-tuned for the closed loop (design_controller.m, J-2.3): with the
% earlier q_v = 200, q_t = 1000 the controller amplified voluntary motion
% at 0.5 Hz by 20-50 %. Possible because the WFLC now gives the frequency:
% a narrower tremor model no longer costs phase lag.
p.q_v  = 1000;         % voluntary  [rad^2/s^3]
p.q_t  = 400;          % tremor     [rad^2/s^3]
p.rho  = 0.9998;       % oscillator damping per step (1 = undamped)

% --- Measurement noise ---------------------------------------------------
% theta_mes comes from accelerometer tilt: noisy AND corrupted by the hand
% acceleration during tremor -> trusted much less than the gyro.
p.sigma_theta = 2*pi/180;  % [rad] std of theta measurement (synthetic)
p.r_theta_scale = 400;     % inflate R_theta: the accelerometer angle of the
                           % hand is corrupted by the hand acceleration
                           % (x3 in the tremor band if the IMU is 2 cm above
                           % the wrist axis, gen_fake_log.m). With 400 the
                           % accelerometer only corrects the gyro drift
                           % (time constant ~1.4 s, < 1 deg error for a
                           % 0.01 rad/s residual gyro bias). Was 25: closed-
                           % loop attenuation fell from 60 % to 45-54 %
                           % (test/host/test_closed_loop.cpp). Check on real
                           % logs with analyze_log (accelerometer gain).
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

% --- Wrist + orthosis model (contracts.h, C4; J-2.1) -----------------------
%   J*theta'' + B*theta' + K*theta = tau_tremor + tau_voluntary + tau_motor
p.plant.J = 1.5e-3;     % [kg m^2] hand + shell inertia
p.plant.K = 0.3;        % [N m/rad] passive stiffness
p.plant.B = 0.02;       % [N m s/rad] damping
p.plant.delay_steps = 3;   % sensor -> torque delay, 3 x 2 ms = 6 ms

% --- Controller (design_controller.m, J-2.3) -------------------------------
% tau = -k_omega(f_hat) * omega_t_hat  ("virtual damper" on the tremor
% velocity only), k_omega interpolated in f_hat. In the 4-gain format of
% contracts.h: K = [0, 0, 0, k_omega].
p.ctrl.f0      = [4 5 6];            % [Hz]
p.ctrl.k_omega = [0.070 0.090 0.105];  % [N m s/rad], from design_controller.m
p.ctrl.tau_max = 0.3;               % [N m] software limit (C6)

% --- Alternative: extended Kalman filter with the frequency as 5th state --
p.ekf.q_w    = 2*(2*pi)^2;  % random-walk intensity of w [rad^2/s^3]
p.ekf.p_w0   = (2*pi*1)^2;  % initial variance of w (1 Hz std)
p.ekf.f_init = 5;
p.ekf.f_min  = 3;
p.ekf.f_max  = 8;
end

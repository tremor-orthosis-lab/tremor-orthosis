%DESIGN_CONTROLLER  Task J-2.3: tremor controller design, Kalman in the loop.
%
% Plant (plant_model.m, J-2.1): J th'' + B th' + K th = tau_trem + tau_vol + tau
% with a 6 ms sensor-to-torque delay. The controller only sees the
% estimate x_hat = [th_v, om_v, th_t, om_t] of the Kalman filter.
%
% Part 1  Textbook LQR with an internal model of the tremor (plant + tremor
%         torque oscillator + delay, full state known). Converted to the
%         equivalent gain on the estimated tremor states and put in the
%         loop with the real Kalman filter -> UNSTABLE. The separation
%         principle does not hold: the Kalman model (motion = voluntary +
%         oscillation) is not the plant model (torque -> inertia).
% Part 2  Design that accounts for the Kalman filter: gain searched on the
%         true closed loop (plant + Kalman + delay), structure
%         tau = -k_omega * omega_t_hat ("virtual damper" acting on the
%         tremor velocity only). Rule: largest k_omega such that the
%         voluntary motion at 0.5 Hz changes by at most 6 % in this linear
%         analysis (criterion of the plan: < 10 %; the time-domain
%         simulation with noise gives ~1.5 % more, so 6 % keeps a margin).
% Part 3  Robustness: Monte-Carlo, 200 draws of J, K, B and of the delay.
%
% Success criteria (plan, J-2.3): attenuation > 50 % at 4, 5 and 6 Hz,
% stable on 100 % of the draws, voluntary motion at 0.5 Hz changed < 10 %.
% Time-domain check with noise, WFLC and saturation: sim_closed_loop.m.

clear; clc; close all;
p  = tremor_params();
pl = p.plant;  nd = pl.delay_steps;  Ts = p.Ts;
F0 = p.ctrl.f0;

%% Part 1 - textbook LQR with internal model, full state
fprintf('Part 1 - LQR with internal tremor model (assumes the full state is known)\n');
pm = plant_model(pl, Ts);
for i = 1:numel(F0)
    f = F0(i); w = 2*pi*f; c = cos(w*Ts); s = sin(w*Ts);
    % state: [theta, omega, d, d_q, u(k-1) ... u(k-nd)], d = tremor torque
    n = 4 + nd;
    A = zeros(n); B = zeros(n,1);
    A(1:2,1:2) = pm.Ad; A(1:2,3) = pm.Bd; A(1:2,4+nd) = pm.Bd;
    A(3:4,3:4) = 0.9998*[c s; -s c];
    for j = 1:nd-1, A(5+j,4+j) = 1; end
    B(5) = 1;
    Q = zeros(n); Q(1,1) = 1;
    Kfull = dlqr_iter(A, B, Q, 0.01);
    % equivalent gain on [theta_t, omega_t] in steady state at f
    zf = exp(1i*w*Ts);
    G  = (zf*eye(2) - pm.Ad) \ pm.Bd;             % [theta; omega] per N m
    a  = Kfull(1) + Kfull(2)*G(2)/G(1);
    bd = Kfull(3) + 1i*Kfull(4);
    cu = sum(Kfull(5:end) .* zf.^(-(1:nd)));
    C  = (a + bd/G(1)) / (1 - bd*zf^(-nd) + cu);   % tau = -C * theta_t
    k_eq = [0 0 real(C) imag(C)/w];
    promised = 1 - abs(1/(1 + C*G(1)*zf^(-nd)));   % if the state were known
    [~, ~, rmax] = loop_metrics(f, k_eq, p, pl, nd);
    verdict = 'UNSTABLE'; if rmax < 1, verdict = 'stable'; end
    fprintf('  %d Hz: promises %3.0f %% attenuation (k_theta = %7.2f, k_omega = %6.3f); with the Kalman filter max|eig| = %.3f -> %s\n', ...
            f, 100*promised, k_eq(3), k_eq(4), rmax, verdict);
end

%% Part 2 - gain search on the true loop
fprintf('\nPart 2 - virtual damper on omega_t_hat, Kalman filter in the loop\n');
kgrid = linspace(0.01, 0.30, 59);
k_omega = zeros(size(F0));
curves = struct([]);
for i = 1:numel(F0)
    A_ = zeros(size(kgrid)); V_ = A_; R_ = A_;
    for j = 1:numel(kgrid)
        [A_(j), V_(j), R_(j)] = loop_metrics(F0(i), [0 0 0 kgrid(j)], p, pl, nd);
    end
    ok = abs(V_) <= 0.06 & R_ < 1;
    jbest = find(ok, 1, 'last');
    k_omega(i) = kgrid(jbest);
    curves(i).att = A_; curves(i).vol = V_; %#ok<SAGROW>
    fprintf('  %d Hz: k_omega = %.3f N m s/rad -> attenuation %3.0f %%, voluntary %+5.1f %%, max|eig| %.4f\n', ...
            F0(i), k_omega(i), 100*A_(jbest), 100*V_(jbest), R_(jbest));
end
% Is a stiffness term useful? Add k_theta around the chosen damper (5 Hz)
fprintf('  5 Hz, adding k_theta:');
for kt = [-1 -0.5 0 0.5 1]
    [a5, v5, r5] = loop_metrics(5, [0 0 kt k_omega(2)], p, pl, nd);
    tag = ''; if r5 >= 1, tag = ' UNSTABLE'; end
    fprintf('  [%+.1f: %2.0f%%, vol %+4.1f%%%s]', kt, 100*a5, 100*v5, tag);
end
fprintf('\n  -> no gain from k_theta: the damper alone is kept.\n');

%% Part 3 - Monte-Carlo robustness
fprintf('\nPart 3 - Monte-Carlo, 200 draws: J +/-30 %%, K +/-50 %%, B +/-50 %%, delay 4-10 ms\n');
rng(1);
Ndraw = 200;
for i = 1:numel(F0)
    att = zeros(Ndraw,1); vol = att; rmax = att;
    for m = 1:Ndraw
        plm.J = pl.J*(0.7 + 0.6*rand);
        plm.K = pl.K*(0.5 + rand);
        plm.B = pl.B*(0.5 + rand);
        ndm = randi([2 5]);
        [att(m), vol(m), rmax(m)] = loop_metrics(F0(i), [0 0 0 k_omega(i)], p, plm, ndm);
    end
    fprintf('  %d Hz: stable %3.0f %% | attenuation min %2.0f %%, median %2.0f %% | voluntary worst %+5.1f %%\n', ...
            F0(i), 100*mean(rmax < 1), 100*min(att), 100*median(att), 100*max(abs(vol)));
end

fprintf('\nGains to copy into tremor_params.m:\n  p.ctrl.k_omega = [%.3f %.3f %.3f];\n', k_omega);
fprintf('C++ (contracts.h, Gains::K, rows f0 = 4, 5, 6 Hz):\n');
for i = 1:numel(F0)
    fprintf('  {0.0f, 0.0f, 0.0f, %.4ff},\n', k_omega(i));
end

%% Figure: attenuation and voluntary change versus the gain
figure('Name','Controller design','Color','w');
subplot(2,1,1); hold on; grid on;
for i = 1:numel(F0), plot(kgrid, 100*curves(i).att, 'LineWidth', 1.5); end
ylabel('tremor attenuation [%]'); yline(50, 'k:');
legend(arrayfun(@(f) sprintf('%d Hz', f), F0, 'UniformOutput', false), 'Location', 'southeast');
title('Virtual damper on the estimated tremor velocity (Kalman in the loop)');
subplot(2,1,2); hold on; grid on;
for i = 1:numel(F0), plot(kgrid, 100*curves(i).vol, 'LineWidth', 1.5); end
yline(10, 'k:'); yline(-10, 'k:');
xlabel('k_\omega [N m s/rad]'); ylabel('voluntary motion at 0.5 Hz [%]');


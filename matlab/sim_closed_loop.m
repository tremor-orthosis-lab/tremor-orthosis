%SIM_CLOSED_LOOP  Tasks J-2.2 / J-2.3: time-domain closed-loop simulation.
%
% Everything that the linear analysis of design_controller.m leaves out:
% sensor noise, the WFLC estimating the frequency online, gains
% interpolated in f_hat, torque saturation at 0.3 N m, 6 ms delay.
%
% Each case is run twice, orthosis passive (tau = 0) and active, with the
% same tremor and voluntary torques:
%   tremor     : torque giving a free tremor of 5 or 10 deg at 4, 5, 6 Hz
%                (amplitude modulated +/-30 % at 0.1 Hz), or a 4->6 Hz chirp
%   voluntary  : torque giving a 20 deg, 0.5 Hz free motion
% Metrics (first 5 s ignored)
%   attenuation : 1 - RMS(2.5-9 Hz band, active) / RMS(same band, passive)
%   voluntary   : change of the 0.5 Hz amplitude, active vs passive
%   torque      : RMS and peak at the wrist, compared with what the motors
%                 can give (J-2.2): 4015 motor 0.12-0.15 N m x ratio 3

clear; clc; close all;
p  = tremor_params();
pl = p.plant;  nd = pl.delay_steps;  Ts = p.Ts;
T  = 30;  t = (0:Ts:T-Ts)';  N = numel(t);  skip = round(5/Ts);
d2r = pi/180;
pm = plant_model(pl, Ts);

cases = { 4, 5;  5, 5;  6, 5;  5, 10;  6, 10;  @(t) 4 + 2*t/T, 5 };
names = {'4 Hz, 5 deg', '5 Hz, 5 deg', '6 Hz, 5 deg', '5 Hz, 10 deg', ...
         '6 Hz, 10 deg', 'chirp 4->6 Hz, 5 deg'};

% band-pass for the metric (zero-phase, offline only): 2nd-order RBJ applied
% forward and backward
bp = @(x) flipud(bandpass_biquad(flipud(bandpass_biquad(x, 2.5, 9, Ts)), 2.5, 9, Ts));
amp05 = @(x) norm([sin(2*pi*0.5*t(skip+1:end)), cos(2*pi*0.5*t(skip+1:end)), ...
                   ones(N-skip,1)] \ x(skip+1:end) .* [1;1;0]);

fprintf('%-22s %12s %11s %10s %10s\n', 'Case', 'attenuation', 'voluntary', 'tau RMS', 'tau peak');
for c = 1:size(cases,1)
    ftr = cases{c,1}; amp = cases{c,2};
    if isa(ftr, 'function_handle'), f = ftr(t); else, f = ftr*ones(N,1); end
    phs = 2*pi*cumsum(f)*Ts;
    th_tr = amp*d2r*(1 + 0.3*sin(2*pi*0.1*t)).*sin(phs);   % free tremor motion
    th_v  = 20*d2r*sin(2*pi*0.5*t);                         % free voluntary motion
    tau_ext = inverse_dynamics(th_tr + th_v, pl, Ts);       % torques that produce them

    for active = [false true]
        rng(1);
        x = [th_tr(1) + th_v(1); 0];
        buf = zeros(nd,1);
        st = estimator_step([], [], p);
        TH = zeros(N,1); U = zeros(N,1); FH = zeros(N,1);
        for k = 1:N
            y = [x(1) + p.sigma_theta*randn, x(2) + p.sigma_omega*randn];
            [st, xh, fh] = estimator_step(st, y, p);
            u = 0;
            if active
                kw = interp1(p.ctrl.f0, p.ctrl.k_omega, min(max(fh, p.ctrl.f0(1)), p.ctrl.f0(end)));
                u = -kw*xh(4);
                u = min(max(u, -p.ctrl.tau_max), p.ctrl.tau_max);
            end
            TH(k) = x(1); U(k) = u; FH(k) = fh;
            buf = [u; buf(1:end-1)];
            x = pm.Ad*x + pm.Bd*(buf(end) + tau_ext(k));
        end
        if active, TH1 = TH; U1 = U; F1 = FH; else, TH0 = TH; end
    end
    att = 1 - std(bp(TH1)) / std(bp(TH0));
    vol = amp05(TH1)/amp05(TH0) - 1;
    fprintf('%-22s %11.0f%% %+10.1f%% %9.3f %10.3f\n', names{c}, 100*att, 100*vol, ...
            std(U1(skip+1:end)), max(abs(U1(skip+1:end))));
    if c == 2, keep.TH0 = TH0; keep.TH1 = TH1; keep.U1 = U1; keep.F1 = F1; end
end
fprintf('\nTorque available at the wrist (J-2.2): 3 x (0.12 to 0.15) = 0.36-0.45 N m,\n');
fprintf('software limit 0.3 N m (C6). A peak at 0.300 means the command saturates.\n');

%% Figure: 5 Hz, 5 deg
figure('Name','Closed loop, 5 Hz','Color','w');
subplot(2,1,1);
plot(t, keep.TH0/d2r, 'Color', [0.6 0.6 0.6]); hold on;
plot(t, keep.TH1/d2r, 'r'); grid on; xlim([10 14]);
ylabel('\theta [deg]'); legend('orthosis passive', 'orthosis active');
title('Simulated wrist angle: voluntary 0.5 Hz + tremor 5 Hz');
subplot(2,1,2);
plot(t, keep.U1, 'b'); grid on; xlim([10 14]);
xlabel('time [s]'); ylabel('\tau [N m]');


function [lag_deg, gain] = phase_fit(est, ref, f, t)
%PHASE_FIT Phase lag and gain of est relative to ref at frequency f.
%   Least-squares fit of a*sin + b*cos at frequency f on both signals.
%   lag_deg > 0 means est lags behind ref.
M  = [sin(2*pi*f*t), cos(2*pi*f*t)];
ce = M \ est;  cr = M \ ref;
pe = atan2(ce(2), ce(1));  pr = atan2(cr(2), cr(1));
lag_deg = angle(exp(1i*(pr - pe)))*180/pi;
gain = norm(ce)/norm(cr);
end

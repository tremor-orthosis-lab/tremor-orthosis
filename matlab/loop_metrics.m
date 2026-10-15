function [att, vol, rmax] = loop_metrics(f0, kgain, p, pl, nd)
%LOOP_METRICS Attenuation, voluntary change and stability of the loop.
%   att  : tremor attenuation at f0, 1 - |theta closed| / |theta open|
%   vol  : change of the voluntary motion at 0.5 Hz, |closed|/|open| - 1
%          (> 0 amplified, < 0 slowed)
%   rmax : largest |eigenvalue| (< 1 = stable)
[M,  Bd] = closed_loop_matrix(f0, kgain, p, pl, nd);
[M0, ~ ] = closed_loop_matrix(f0, zeros(1,4), p, pl, nd);
r  = @(MM, f) [1 zeros(1,size(MM,1)-1)] * ((exp(1i*2*pi*f*p.Ts)*eye(size(MM,1)) - MM) \ Bd);
att  = 1 - abs(r(M, f0)/r(M0, f0));
vol  = abs(r(M, 0.5)/r(M0, 0.5)) - 1;
rmax = max(abs(eig(M)));
end

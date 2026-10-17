function write_log(path, t, acc1, gyr1, acc2, gyr2, tau_cmd)
%WRITE_LOG Write a log in the firmware CSV format (C3), for simulated data.
%   Estimator columns are written as 0, vbat 0, loop_us 400.
N = numel(t);
if nargin < 7, tau_cmd = zeros(N,1); end
n = (1:N)';
t_us = round(1e6 + t(:)*1e6);
z = zeros(N,1);
D = [n, t_us, acc1, gyr1, acc2, gyr2, z, z, z, z, tau_cmd(:), z, 400*ones(N,1)];
fid = fopen(path, 'w');
fprintf(fid, 'n,t_us,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,theta_hat,theta_v_hat,theta_t_hat,f_hat,tau_cmd,vbat,loop_us\n');
fprintf(fid, ['%d,%d,' repmat('%.5f,', 1, 18) '%d\n'], D');
fclose(fid);
end

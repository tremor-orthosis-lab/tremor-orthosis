function L = load_log(path)
%LOAD_LOG Read a firmware log (CSV, format C3) into a struct (task J-1.4).
%   L = load_log(path)
%   Every column becomes a field (L.ax1, L.gy2, L.theta_t_hat, ...), plus:
%     L.t          time [s] from the first sample (t_us wrap-around handled)
%     L.theta_mes  wrist angle from the accelerometers [rad] (same formula
%                  as the firmware, src/control/controller.cpp)
%     L.omega_mes  wrist rate from the gyros, gy1 - gy2 [rad/s]
%     L.lost       number of lost samples (gaps in the counter n)
%     L.fs         measured sample rate [Hz]
%     L.name       file name without extension
%   Lines starting with '#' are ignored.
fid = fopen(path, 'r');
if fid < 0, error('load_log: cannot open %s', path); end
header = strsplit(strtrim(fgetl(fid)), ',');
C = textscan(fid, repmat('%f', 1, numel(header)), 'Delimiter', ',', ...
             'CommentStyle', '#', 'CollectOutput', true);
fclose(fid);
D = C{1};
D = D(all(~isnan(D), 2), :);              % drop a truncated last line
L = struct();
for k = 1:numel(header)
    L.(header{k}) = D(:,k);
end
[~, L.name] = fileparts(path);

dt_us = mod(diff(L.t_us), 2^32);          % micros() wraps after 71 min
L.t   = [0; cumsum(dt_us)]/1e6;
L.lost = sum(diff(L.n) - 1);
L.fs  = (numel(L.t) - 1)/L.t(end);

a1 = atan2(-L.ax1, L.az1);
a2 = atan2(-L.ax2, L.az2);
L.theta_mes = angle(exp(1i*(a1 - a2)));   % wrapped to [-pi, pi]
L.omega_mes = L.gy1 - L.gy2;
end

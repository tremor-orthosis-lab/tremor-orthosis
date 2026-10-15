function tau = inverse_dynamics(th, pl, Ts)
%INVERSE_DYNAMICS Torque that makes the passive wrist follow th exactly.
%   tau = inverse_dynamics(th, pl, Ts) = J*th'' + B*th' + K*th
%   Used by sim_closed_loop.m to build tremor and voluntary torques that
%   give a chosen free motion.
om = gradient(th, Ts);
al = gradient(om, Ts);
tau = pl.J*al + pl.B*om + pl.K*th;
end

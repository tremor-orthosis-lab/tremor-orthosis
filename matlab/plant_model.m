function pm = plant_model(pl, Ts)
%PLANT_MODEL Wrist + orthosis model (task J-2.1).
%   pm = plant_model(pl, Ts)
%     pl : struct with fields J [kg m^2], K [N m/rad], B [N m s/rad]
%          (default values in tremor_params.m, p.plant = contract C4)
%     Ts : sample time [s]
%   Model:  J*theta'' + B*theta' + K*theta = tau_tremor + tau_vol + tau_motor
%           state [theta; omega], input = total torque at the wrist [N m]
%   pm.Ad, pm.Bd : exact zero-order-hold discretisation
%   pm.Z(f)      : torque needed per radian of motion at f [Hz] (complex),
%                  Z = K - J*w^2 + 1i*B*w
%   pm.T0(f,a)   : tremor torque amplitude [N m] that gives a free
%                  (orthosis passive) tremor of a [deg] at f [Hz]
%
%   With the nominal values the wrist is inertia-dominated above
%   f_n = sqrt(K/J)/(2 pi) = 2.3 Hz: at tremor frequencies, the torque
%   mostly accelerates the hand.

J = pl.J; K = pl.K; B = pl.B;
Ac = [0 1; -K/J -B/J];
Bc = [0; 1/J];
E  = expm([Ac Bc; 0 0 0]*Ts);
pm.Ad = E(1:2,1:2);
pm.Bd = E(1:2,3);
pm.Ac = Ac; pm.Bc = Bc;
pm.Z  = @(f) K - J*(2*pi*f).^2 + 1i*B*2*pi*f;
pm.T0 = @(f, a_deg) abs(pm.Z(f)) .* a_deg*pi/180;
pm.f_n = sqrt(K/J)/(2*pi);
end

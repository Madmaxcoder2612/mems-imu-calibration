function b = gyro_thermal_bias(P, T, dTdt)
%GYRO_THERMAL_BIAS  True temperature-dependent gyro bias (deg/s).
%   T    : 1xN sensor temperature (degC)
%   dTdt : 1xN temperature rate (degC/min)
%   b    : 3xN bias
%   Cubic drift in temperature plus a hysteresis term: the bias differs
%   between heating and cooling (thermal stress in the MEMS package), and
%   the hysteresis grows slightly with temperature.
dT = T(:).' - 25;
r  = dTdt(:).';
G  = P.gyr;
b  = G.c0 + G.c1.*dT + G.c2.*dT.^2 + G.c3.*dT.^3 ...
     + G.h .* (1 + 0.01*dT) .* tanh(r/0.5);
end

function [tau, adev] = allan_deviation(omega, fs, nPts)
%ALLAN_DEVIATION  Overlapping Allan deviation of a rate signal.
%   omega : static gyro output (any unit, e.g. deg/s), fs : Hz
%   Cluster times are log-spaced up to T/9 (IEEE Std 952 guidance).
omega = omega(:);  N = numel(omega);
theta = [0; cumsum(omega)]/fs;                   % integrated angle
m     = unique(round(logspace(0, log10(floor(N/9)), nPts)));
tau   = m/fs;
adev  = zeros(size(m));
for k = 1:numel(m)
    mk = m(k);
    d  = theta(1+2*mk:end) - 2*theta(1+mk:end-mk) + theta(1:end-2*mk);
    adev(k) = sqrt(sum(d.^2) / (2*tau(k)^2*(N + 1 - 2*mk)));
end
end

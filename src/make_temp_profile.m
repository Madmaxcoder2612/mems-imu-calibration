function [t, T] = make_temp_profile(Tlist, rate, holdMin, fs, lagTau)
%MAKE_TEMP_PROFILE  Climate-chamber temperature profile seen by the sensor.
%   Tlist   : chamber set points (degC), visited in order
%   rate    : ramp rate (degC/min), scalar or one value per ramp
%   holdMin : dwell time at each set point (min)
%   fs      : sample rate (Hz)
%   lagTau  : thermal lag between chamber and sensor die (s)
tw = 0;  Tw = Tlist(1);
if isscalar(rate), rate = rate*ones(1, numel(Tlist) - 1); end
for k = 2:numel(Tlist)
    tw(end+1) = tw(end) + abs(Tlist(k) - Tlist(k-1))/rate(k-1)*60;  %#ok<AGROW>
    Tw(end+1) = Tlist(k);                                       %#ok<AGROW>
    if holdMin > 0
        tw(end+1) = tw(end) + holdMin*60;                       %#ok<AGROW>
        Tw(end+1) = Tlist(k);                                   %#ok<AGROW>
    end
end
t = 0:1/fs:tw(end);
T = interp1(tw, Tw, t);
a = exp(-1/(fs*lagTau));                       % first-order thermal lag
T = filter(1 - a, [1 -a], T, a*T(1));
end

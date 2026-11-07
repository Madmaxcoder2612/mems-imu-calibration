function q = triad_quat(f, m, ref)
%TRIAD_QUAT  Orientation from one accelerometer + magnetometer sample (TRIAD).
%   Assumes the accelerometer measures only gravity (no linear acceleration)
%   and the magnetometer measures only the Earth field.
t1b = f/norm(f);           t1n = ref.up;
t2b = cross(t1b, m);       t2n = cross(t1n, ref.m);
t2b = t2b/norm(t2b);       t2n = t2n/norm(t2n);
t3b = cross(t1b, t2b);     t3n = cross(t1n, t2n);
q = R2q([t1n t2n t3n] * [t1b t2b t3b].');
end

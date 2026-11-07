function q = quat_slerp(a, b, t)
%QUAT_SLERP  Spherical linear interpolation from quaternion a to b (0 <= t <= 1).
d = a.'*b;
if d < 0, b = -b; d = -d; end          % take the short way round
if d > 0.9995
    q = a + t*(b - a);
else
    th = acos(d);
    q  = (sin((1 - t)*th)*a + sin(t*th)*b)/sin(th);
end
q = q/norm(q);
end

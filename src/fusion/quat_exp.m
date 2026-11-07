function q = quat_exp(v)
%QUAT_EXP  Quaternion of a rotation vector v (rad).
th = norm(v);
if th < 1e-12
    q = [1; v(:)/2];
    q = q/norm(q);
else
    q = [cos(th/2); sin(th/2)*v(:)/th];
end
end

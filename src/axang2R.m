function R = axang2R(v)
%AXANG2R  Rotation matrix from a rotation vector v (rad), Rodrigues formula.
th = norm(v);
S  = @(u) [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
if th < 1e-12
    R = eye(3) + S(v);
    return
end
K = S(v/th);
R = eye(3) + sin(th)*K + (1 - cos(th))*(K*K);
end

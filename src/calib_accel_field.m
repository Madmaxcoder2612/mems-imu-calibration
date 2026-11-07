function [T, b, info] = calib_accel_field(Y, g)
%CALIB_ACCEL_FIELD  Turntable-free accelerometer calibration.
%   Uses only the fact that a static accelerometer must measure |f| = g,
%   so the orientations do not need to be known (e.g. sensor placed by
%   hand in ~20 different poses).
%   Inverse model:  f_hat = T*(y - b),  T upper-triangular -> 9 parameters.
%   (A full 3x3 T is not identifiable: any rotation keeps |f| unchanged.)
%   Solved with Levenberg-Marquardt and a finite-difference Jacobian.
p      = [1 1 1 0 0 0 0 0 0].';    % [T11 T22 T33 T12 T13 T23 bx by bz]
lambda = 1e-3;
fun    = @(p) residuals(p, Y, g);
r      = fun(p);
cost   = r.'*r;
for it = 1:200
    J  = finite_diff_jacobian(fun, p, r);
    A  = J.'*J;
    dp = -(A + lambda*diag(diag(A))) \ (J.'*r);
    rNew = fun(p + dp);  costNew = rNew.'*rNew;
    if costNew < cost
        converged = (cost - costNew) < 1e-12*max(cost, eps);
        p = p + dp;  r = rNew;  cost = costNew;  lambda = lambda/3;
        if converged, break; end
    else
        lambda = lambda*5;
        if lambda > 1e10, break; end
    end
end
[T, b]          = unpack(p);
info.iterations = it;
info.rms        = sqrt(cost/numel(r));
end

function [T, b] = unpack(p)
T = [p(1) p(4) p(5); 0 p(2) p(6); 0 0 p(3)];
b = p(7:9);
end

function r = residuals(p, Y, g)
[T, b] = unpack(p);
F = T*(Y - b);
r = sqrt(sum(F.^2, 1)).' - g;
end

function J = finite_diff_jacobian(fun, p, r0)
J = zeros(numel(r0), numel(p));
for k = 1:numel(p)
    h  = 1e-6*max(1, abs(p(k)));
    pk = p;  pk(k) = pk(k) + h;
    J(:, k) = (fun(pk) - r0)/h;
end
end

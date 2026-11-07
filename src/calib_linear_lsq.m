function [K, b, resid] = calib_linear_lsq(Y, X)
%CALIB_LINEAR_LSQ  Estimate y = K*x + b from known references.
%   Y : 3xN measured outputs (e.g. averaged static/rate-table data)
%   X : 3xN known inputs (gravity vector or table rate in sensor frame)
%   Solves 12 parameters (9 in K: scale factor + misalignment, 3 bias)
%   with one linear least-squares step.
Phi   = [X; ones(1, size(X,2))];   % 4xN regressor
Theta = Y / Phi;                   % 3x4 = [K b]
K     = Theta(:, 1:3);
b     = Theta(:, 4);
resid = Y - Theta*Phi;
end

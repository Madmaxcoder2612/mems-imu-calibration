function Q = filt_gyro_only(gyr, acc, mag, dt, ref)
%FILT_GYRO_ONLY  Dead reckoning: integrate the gyro only (baseline).
%   Initial orientation from TRIAD; no correction afterwards, so errors grow.
N = size(gyr, 2);  Q = zeros(4, N);
q = triad_quat(acc(:,1), mag(:,1), ref);
for k = 1:N
    q = quat_mult(q, quat_exp(gyr(:,k)*dt));
    Q(:, k) = q;
end
end

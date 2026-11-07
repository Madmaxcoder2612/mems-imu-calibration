function Q = filt_complementary(gyr, acc, mag, dt, ref, alpha)
%FILT_COMPLEMENTARY  Quaternion complementary filter.
%   High-pass the gyro (integration) and low-pass the accel/mag orientation
%   (TRIAD) by blending the two with a small SLERP step ALPHA each sample.
%   Time constant ~ dt/alpha.
N = size(gyr, 2);  Q = zeros(4, N);
q = triad_quat(acc(:,1), mag(:,1), ref);
for k = 1:N
    q = quat_mult(q, quat_exp(gyr(:,k)*dt));
    q = quat_slerp(q, triad_quat(acc(:,k), mag(:,k), ref), alpha);
    Q(:, k) = q;
end
end

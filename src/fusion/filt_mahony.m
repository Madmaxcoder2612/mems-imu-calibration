function [Q, bHat] = filt_mahony(gyr, acc, mag, dt, ref, Kp, Ki)
%FILT_MAHONY  Mahony nonlinear complementary filter (MARG version).
%   Error = cross product between measured and predicted gravity / magnetic
%   directions. A PI controller feeds it back into the gyro rate; the
%   integral term estimates the gyro bias (bHat = -Ki * integral).
%   Ref: Mahony, Hamel & Pflimlin, IEEE Trans. Automatic Control, 2008.
N = size(gyr, 2);  Q = zeros(4, N);  bHat = zeros(3, N);
q = triad_quat(acc(:,1), mag(:,1), ref);
eInt = zeros(3, 1);
for k = 1:N
    a = acc(:,k)/norm(acc(:,k));
    m = mag(:,k)/norm(mag(:,k));
    R = q2R(q);
    h = R*m;                                  % magnetic field in nav frame
    b = [hypot(h(1), h(2)); 0; h(3)];         % remove the heading part
    v = R.'*ref.up;                           % predicted gravity direction
    w = R.'*b;                                % predicted field direction
    e = cross(a, v) + cross(m, w);
    eInt = eInt + e*dt;
    omega = gyr(:,k) + Kp*e + Ki*eInt;
    q = q + 0.5*quat_mult(q, [0; omega])*dt;
    q = q/norm(q);
    Q(:, k) = q;  bHat(:, k) = -Ki*eInt;
end
end

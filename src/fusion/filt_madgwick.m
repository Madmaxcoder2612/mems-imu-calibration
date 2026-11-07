function Q = filt_madgwick(gyr, acc, mag, dt, ref, beta)
%FILT_MADGWICK  Madgwick gradient-descent orientation filter (MARG version).
%   Each step corrects the gyro-integrated quaternion with one normalised
%   gradient-descent step that aligns predicted and measured gravity and
%   magnetic directions. BETA is the correction gain (rad/s).
%   Ref: Madgwick, Harrison & Vaidyanathan, IEEE ICORR, 2011.
N = size(gyr, 2);  Q = zeros(4, N);
q = triad_quat(acc(:,1), mag(:,1), ref);
for k = 1:N
    a = acc(:,k)/norm(acc(:,k));
    m = mag(:,k)/norm(mag(:,k));
    R = q2R(q);
    h = R*m;
    bx = hypot(h(1), h(2));  bz = h(3);
    q1 = q(1);  q2 = q(2);  q3 = q(3);  q4 = q(4);
    F = [R.'*ref.up - a;  R.'*[bx; 0; bz] - m];        % objective function
    J = [-2*q3,             2*q4,             -2*q1,                2*q2;
          2*q2,             2*q1,              2*q4,                2*q3;
          0,               -4*q2,             -4*q3,                0;
         -2*bz*q3,          2*bz*q4,          -4*bx*q3-2*bz*q1,    -4*bx*q4+2*bz*q2;
         -2*bx*q4+2*bz*q2,  2*bx*q3+2*bz*q1,   2*bx*q2+2*bz*q4,    -2*bx*q1+2*bz*q3;
          2*bx*q3,          2*bx*q4-4*bz*q2,   2*bx*q1-4*bz*q3,     2*bx*q2];
    step = J.'*F;
    step = step/norm(step);
    q = q + (0.5*quat_mult(q, [0; gyr(:,k)]) - beta*step)*dt;
    q = q/norm(q);
    Q(:, k) = q;
end
end

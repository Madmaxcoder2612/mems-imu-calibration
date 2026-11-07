function [Q, bHat] = filt_mekf(gyr, acc, mag, dt, ref, prm)
%FILT_MEKF  Adaptive multiplicative extended Kalman filter (error-state EKF).
%   State: quaternion q (kept outside the filter) + gyro bias b.
%   Error state (6): small body-frame rotation dtheta and bias error db.
%   - Predict with the bias-corrected gyro; propagate the 6x6 covariance.
%   - Update with normalised accelerometer and magnetometer directions.
%   - Adaptive noise: the accelerometer is trusted less when |f| differs
%     from g (linear acceleration), the magnetometer less when its norm or
%     dip angle differ from the Earth field (magnetic disturbance).
%   - The magnetometer only corrects heading, so disturbances cannot tilt
%     the estimate.
%   Ref: Markley, "Attitude error representations for Kalman filtering",
%        J. Guidance, Control and Dynamics, 2003.
N  = size(gyr, 2);  Q = zeros(4, N);  bHat = zeros(3, N);
q  = triad_quat(acc(:,1), mag(:,1), ref);
b  = zeros(3, 1);
P  = blkdiag(prm.P0_att^2*eye(3), prm.P0_bias^2*eye(3));
Qd = blkdiag(prm.sig_g^2*dt*eye(3), prm.sig_b^2*dt*eye(3));
I6 = eye(6);
for k = 1:N
    % ---- predict ----
    w = gyr(:,k) - b;
    q = quat_mult(q, quat_exp(w*dt));
    F = [q2R(quat_exp(-w*dt)), -eye(3)*dt;  zeros(3), eye(3)];
    P = F*P*F.' + Qd;

    % ---- update ----
    a  = acc(:,k);  m = mag(:,k);
    na = norm(a);   nm = norm(m);
    R  = q2R(q);
    u  = R.'*ref.up;                         % predicted gravity direction
    mp = R.'*ref.m;                          % predicted field direction
    z  = [a/na; m/nm];
    hx = [u; mp];
    H  = [skew3(u),          zeros(3);
          skew3(mp)*(u*u.'), zeros(3)];      % mag: heading correction only
    dip = asin(max(-1, min(1, -(m/nm).'*(a/na))));
    ra  = prm.sig_a^2*(1 + (abs(na - ref.g)/ref.g/0.01)^2);
    rm  = prm.sig_m^2*(1 + (abs(nm - 1)/0.01)^2 + (abs(dip - ref.dip)/(pi/180))^2);
    Rm  = blkdiag(ra*eye(3), rm*eye(3));
    S   = H*P*H.' + Rm;
    K   = (P*H.')/S;
    dx  = K*(z - hx);
    q   = quat_mult(q, [1; dx(1:3)/2]);
    q   = q/norm(q);
    b   = b + dx(4:6);
    IKH = I6 - K*H;
    P   = IKH*P*IKH.' + K*Rm*K.';            % Joseph form, stays symmetric

    Q(:, k) = q;  bHat(:, k) = b;
end
end

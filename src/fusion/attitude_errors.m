function E = attitude_errors(Q, Rt)
%ATTITUDE_ERRORS  Error between estimated quaternions and true orientation.
%   Q  : 4xN estimates, Q(:,k) is the estimate after step k
%   Rt : 3x3x(N+1) true rotation matrices
%   E.total : 1xN total rotation error (deg)
%   E.rpy   : 3xN roll/pitch/yaw error (deg)
N = size(Q, 2);
E.total = zeros(1, N);  E.rpy = zeros(3, N);
for k = 1:N
    dR = Rt(:,:,k+1).' * q2R(Q(:,k));
    E.total(k)  = acos(max(-1, min(1, (trace(dR) - 1)/2)))*180/pi;
    E.rpy(:, k) = R2eul_zyx(dR)*180/pi;
end
end

function S = sim_fusion_truth(fs, Tend, g)
%SIM_FUSION_TRUTH  True motion, specific force and magnetic field for the
%   sensor-fusion test. Navigation frame: x = magnetic north, y = west, z = up.
%   Includes linear-acceleration bursts and one magnetic disturbance, which
%   violate the assumptions that accelerometer = gravity and mag = Earth field.
d2r = pi/180;  dt = 1/fs;  N = round(Tend*fs);
t   = (0:N)*dt;

% Smooth orientation profile (ZYX Euler angles)
roll  = (35*sin(2*pi*0.05*t)  + 10*sin(2*pi*0.31*t))*d2r;
pitch = (25*sin(2*pi*0.037*t + 1) + 8*sin(2*pi*0.23*t))*d2r;
yaw   = (120*sin(2*pi*0.011*t) + 15*sin(2*pi*0.17*t))*d2r;
Rt = zeros(3, 3, N+1);
for k = 1:N+1
    Rt(:,:,k) = eul2R_zyx(roll(k), pitch(k), yaw(k));
end
% Body rates consistent with the orientation samples (exact over each step)
w = zeros(3, N);
for k = 1:N
    w(:, k) = rot_log(Rt(:,:,k).'*Rt(:,:,k+1))/dt;
end

% Earth field (Stuttgart: inclination ~64 deg), unit length
dip = 64*d2r;
ref.up  = [0; 0; 1];
ref.m   = [cos(dip); 0; -sin(dip)];
ref.dip = dip;
ref.g   = g;

% Disturbances
tk = t(1:N);
aN = zeros(3, N);                                   % linear acceleration, nav
bursts = [40 2.5 1; 90 -2.0 2; 150 3.0 1; 230 2.0 2; 270 -2.5 1];  % [t, m/s^2, axis]
for i = 1:size(bursts, 1)
    aN(bursts(i,3), :) = aN(bursts(i,3), :) + bursts(i,2)*exp(-0.5*((tk - bursts(i,1))/1.0).^2);
end
mD = zeros(3, N);                                   % magnetic disturbance, nav
magWin = tk > 190 & tk < 215;
mD(:, magWin) = repmat([0.15; 0.20; 0.25], 1, nnz(magWin));

f = zeros(3, N);  m = zeros(3, N);
for k = 1:N
    f(:, k) = Rt(:,:,k).' * (aN(:,k) + [0; 0; g]);   % specific force, body
    m(:, k) = Rt(:,:,k).' * (ref.m + mD(:,k));      % magnetic field, body
end

burstWin = false(1, N);
for i = 1:size(bursts, 1)
    burstWin = burstWin | abs(tk - bursts(i,1)) < 3;
end

S.t = tk;  S.dt = dt;  S.N = N;  S.Rt = Rt;  S.w = w;  S.f = f;  S.m = m;
S.ref = ref;  S.burstWin = burstWin;  S.magWin = magWin;
S.disturbed = burstWin | (tk > 190 & tk < 225);     % incl. recovery time
end

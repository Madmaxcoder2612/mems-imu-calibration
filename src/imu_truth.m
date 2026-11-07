function P = imu_truth()
%IMU_TRUTH  "Ground-truth" error parameters of the simulated MEMS IMU.
%   These values are hidden from the calibration algorithms. They are only
%   used to generate synthetic sensor data and to score the estimates.
%   Magnitudes are typical for a consumer-grade 6-axis MEMS IMU.

P.g  = 9.80665;          % standard gravity, m/s^2
P.fs = 100;              % IMU output data rate, Hz

% ---------------- Accelerometer:  y = Ka*f + ba + noise -----------------
sa = [ 0.020; -0.015;  0.030];            % scale-factor errors (-)
Ma = [ 0       0.004  -0.006;             % misalignment / cross-axis (rad)
      -0.003   0       0.005;
       0.007  -0.002   0     ];
P.acc.K  = (eye(3) + diag(sa)) * (eye(3) + Ma);
P.acc.b  = [0.05; -0.08; 0.12];           % bias, m/s^2
P.acc.nd = 150e-6 * P.g;                  % noise density, m/s^2/sqrt(Hz)

% -------- Gyroscope:  y = Kg*w + bg(T, dT/dt) + b_GM + noise -----------
sg = [ 0.010; -0.020;  0.015];
Mg = [ 0      -0.005   0.003;
       0.004   0      -0.006;
      -0.002   0.006   0     ];
P.gyr.K  = (eye(3) + diag(sg)) * (eye(3) + Mg);

% Thermal bias model (deg/s), dT = T - 25 degC  -> see gyro_thermal_bias.m
P.gyr.c0 = [ 0.50; -0.30;  0.80];
P.gyr.c1 = [ 4e-3; -3e-3;  5e-3];
P.gyr.c2 = [ 5e-5;  8e-5; -6e-5];
P.gyr.c3 = [ 1e-6; -1e-6;  2e-6];
P.gyr.h  = [ 0.030; -0.020; 0.040];       % thermal hysteresis amplitude

% Stochastic errors
P.gyr.arw      = 0.01;    % angle random walk, deg/s/sqrt(Hz) (= 0.6 deg/sqrt(h))
P.gyr.gm_sigma = 0.003;   % Gauss-Markov bias wander, deg/s
P.gyr.gm_tau   = 200;     % Gauss-Markov correlation time, s
end

%% ========================================================================
%  MEMS IMU SENSOR FUSION - COMPARISON OF ORIENTATION FILTERS
%  ------------------------------------------------------------------------
%  Estimates 3-D orientation from gyro + accelerometer + magnetometer with
%  five methods, on raw and on calibrated sensor data:
%    1. Gyro-only integration    (baseline, no correction)
%    2. Complementary filter     (quaternion SLERP blend with TRIAD)
%    3. Mahony filter            (PI feedback, estimates gyro bias)
%    4. Madgwick filter          (gradient descent)
%    5. Adaptive error-state EKF (estimates gyro bias, rejects disturbances)
%
%  Scenario: 5 min of motion at 45 degC with linear-acceleration bursts and
%  a magnetic disturbance (e.g. passing a steel structure).
%
%  Run main_mems_calibration.m first: this script loads its calibration.
%  Requirements: base MATLAB R2016b+, no toolboxes.
%  ========================================================================
clear; close all; clc;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, 'src')));
outDir  = fullfile(thisDir, 'results');
calFile = fullfile(outDir, 'calibration_params.mat');
if ~exist(calFile, 'file')
    error('Calibration not found. Run main_mems_calibration.m first.');
end
C = load(calFile);

rng(7);
P    = imu_truth();
fs   = P.fs;
d2r  = pi/180;
rmsv = @(x) sqrt(mean(x(:).^2));

%% 1) True motion and disturbances ----------------------------------------
S = sim_fusion_truth(fs, 300, P.g);
N = S.N;

%% 2) Sensor outputs: raw and calibrated ----------------------------------
Tamb   = 45*ones(1, N);                                   % warm environment
gyrRaw = P.gyr.K*(S.w/d2r) + gyro_thermal_bias(P, Tamb, zeros(1, N)) ...
       + gauss_markov(N, fs, P.gyr.gm_sigma, P.gyr.gm_tau, 3) ...
       + P.gyr.arw*sqrt(fs)*randn(3, N);                  % deg/s
accRaw = P.acc.K*S.f + P.acc.b + P.acc.nd*sqrt(fs)*randn(3, N);
mag    = S.m + 0.005*randn(3, N);    % magnetometer assumed hard/soft-iron calibrated

gyrCal = C.Kg_hat \ (gyrRaw - mlp_predict(C.net, [Tamb; zeros(1, N)]));
accCal = C.Ka_hat \ (accRaw - C.ba_hat);

sets = struct('name', {'Raw', 'Calibrated'}, ...
              'gyr',  {gyrRaw*d2r, gyrCal*d2r}, ...       % filters use rad/s
              'acc',  {accRaw, accCal});

%% 3) Filter settings -----------------------------------------------------
alpha = 0.005;          % complementary: time constant dt/alpha = 2 s
Kp = 1.0;  Ki = 0.05;   % Mahony
beta  = 0.04;           % Madgwick, rad/s
prm.sig_g   = P.gyr.arw*d2r;    % gyro white noise, rad/s/sqrt(Hz)
prm.sig_b   = 1e-5;             % bias random walk, rad/s/sqrt(s)
prm.sig_a   = 0.02;             % accel direction noise (normalised)
prm.sig_m   = 0.02;             % mag direction noise (normalised)
prm.P0_att  = 2*d2r;            % initial attitude std
prm.P0_bias = 1*d2r;            % initial bias std (covers an uncalibrated gyro)

names = {'Gyro only', 'Complementary', 'Mahony', 'Madgwick', 'Adaptive EKF'};
nF    = numel(names);

%% 4) Run all filters on both data sets ----------------------------------
res = cell(2, nF);
for s = 1:2
    g = sets(s).gyr;  a = sets(s).acc;  dt = S.dt;  ref = S.ref;
    fprintf('Running filters on %s data...\n', sets(s).name);
    Q = cell(1, nF);
    Q{1}           = filt_gyro_only(g, a, mag, dt, ref);
    Q{2}           = filt_complementary(g, a, mag, dt, ref, alpha);
    [Q{3}, bMah]   = filt_mahony(g, a, mag, dt, ref, Kp, Ki);
    Q{4}           = filt_madgwick(g, a, mag, dt, ref, beta);
    [Q{5}, bEkf]   = filt_mekf(g, a, mag, dt, ref, prm);
    for f = 1:nF
        res{s, f} = attitude_errors(Q{f}, S.Rt);
    end
    sets(s).bMah = bMah;  sets(s).bEkf = bEkf;
end

% Metrics
quiet = ~S.disturbed;
[rmsAll, rmsQuiet, maxDist] = deal(zeros(2, nF));
for s = 1:2
    for f = 1:nF
        e = res{s, f}.total;
        rmsAll(s, f)   = rmsv(e);
        rmsQuiet(s, f) = rmsv(e(quiet));
        maxDist(s, f)  = max(e(S.disturbed));
    end
end

%% 5) Figures -------------------------------------------------------------
% Fig 6: attitude error over time, calibrated data
figure('Name', 'Fusion: attitude error', 'Position', [60 60 1100 450]);
yl = [1e-3 1e3];
shade_windows(S.t, S.burstWin, yl, [1 0.85 0.85]);  hold on;
shade_windows(S.t, S.magWin,   yl, [0.85 0.9 1]);
h = gobjects(1, nF);
for f = 1:nF
    h(f) = semilogy(S.t, res{2, f}.total, 'LineWidth', 1 + (f == nF));   % EKF thicker
end
set(gca, 'YScale', 'log');  ylim(yl);  xlim([0 S.t(end)]);  grid on;
xlabel('Time (s)');  ylabel('Attitude error (deg)');
legend(h, names, 'Location', 'northwest', 'NumColumns', 3);
title('Orientation error on calibrated data (red: linear acceleration, blue: magnetic disturbance)');
saveas(gcf, fullfile(outDir, 'fig6_fusion_attitude_error.png'));

% Fig 7: raw vs calibrated, quiet vs disturbed
figure('Name', 'Fusion: summary', 'Position', [80 80 1100 420]);
subplot(1, 2, 1);
bar(rmsQuiet.');  set(gca, 'YScale', 'log', 'XTickLabel', names);  ylim([0.1 1000]);  grid on;
bar_labels(rmsQuiet);
ylabel('RMS attitude error (deg)');  legend('Raw sensors', 'Calibrated sensors', 'Location', 'northeast');
title('Undisturbed periods');  xtickangle(20);
subplot(1, 2, 2);
bar(maxDist.');  set(gca, 'YScale', 'log', 'XTickLabel', names);  ylim([0.1 1000]);  grid on;
bar_labels(maxDist);
ylabel('Max attitude error (deg)');  legend('Raw sensors', 'Calibrated sensors', 'Location', 'northeast');
title('During disturbances');  xtickangle(20);
saveas(gcf, fullfile(outDir, 'fig7_fusion_summary.png'));

% Fig 8: online gyro-bias estimation on RAW data
bTrue = gyro_thermal_bias(P, 45, 0);                     % deg/s
figure('Name', 'Fusion: bias estimation', 'Position', [100 100 1100 380]);
axL = {'x', 'y', 'z'};
for i = 1:3
    subplot(1, 3, i);
    plot(S.t, sets(1).bMah(i, :)/d2r, S.t, sets(1).bEkf(i, :)/d2r, 'LineWidth', 1); hold on;
    plot(S.t([1 end]), bTrue(i)*[1 1], 'k--', 'LineWidth', 1.2);
    grid on; xlabel('Time (s)'); ylabel('Gyro bias (deg/s)');
    title(sprintf('%s-axis', axL{i}));
    if i == 1, legend('Mahony', 'Adaptive EKF', 'True bias (45 degC)', 'Location', 'best'); end
end
sgtitle('Online gyro-bias estimation from uncalibrated (raw) data');
saveas(gcf, fullfile(outDir, 'fig8_bias_estimation.png'));

%% 6) Results summary -----------------------------------------------------
fprintf('\n=============== SENSOR FUSION RESULTS (deg) ===============\n');
fprintf('%-15s | %16s | %16s | %16s\n', '', 'RMS, whole run', 'RMS, undisturbed', 'Max, disturbed');
fprintf('%-15s | %7s %8s | %7s %8s | %7s %8s\n', 'Filter', 'raw', 'cal', 'raw', 'cal', 'raw', 'cal');
for f = 1:nF
    fprintf('%-15s | %7.2f %8.2f | %7.2f %8.2f | %7.2f %8.2f\n', names{f}, ...
            rmsAll(1, f), rmsAll(2, f), rmsQuiet(1, f), rmsQuiet(2, f), maxDist(1, f), maxDist(2, f));
end
fprintf('Final EKF bias estimate (raw data): [%s] deg/s, true [%s]\n', ...
        sprintf('%.3f ', sets(1).bEkf(:, end)/d2r), sprintf('%.3f ', bTrue));
fprintf('Figures saved to: %s\n', outDir);

%% ------------------------------------------------------------------------
function shade_windows(t, mask, yl, col)
%SHADE_WINDOWS  Draw shaded background patches where MASK is true.
d  = diff([0 mask 0]);
i0 = find(d == 1);  i1 = find(d == -1) - 1;
for k = 1:numel(i0)
    patch([t(i0(k)) t(i1(k)) t(i1(k)) t(i0(k))], [yl(1) yl(1) yl(2) yl(2)], col, ...
          'EdgeColor', 'none', 'HandleVisibility', 'off');
    hold on;
end
end

function bar_labels(vals)
%BAR_LABELS  Print values above a grouped bar chart (2 rows: raw, calibrated).
offs = [-0.15 0.15];
for r = 1:size(vals, 1)
    for c = 1:size(vals, 2)
        text(c + offs(r), vals(r, c)*1.3, sprintf('%.3g', vals(r, c)), ...
             'HorizontalAlignment', 'center', 'FontSize', 7);
    end
end
end

%% ========================================================================
%  MEMS IMU CALIBRATION & SYSTEM IDENTIFICATION - SIMULATION STUDY
%  ------------------------------------------------------------------------
%  1. Accelerometer lab calibration       (known orientations, linear LS)
%  2. Accelerometer field calibration     (unknown orientations, LM on |f|=g)
%  3. Gyroscope rate-table calibration    (scale factor, misalignment, bias)
%  4. Gyro thermal-drift compensation     (polynomial vs. features vs. MLP)
%  5. Allan-variance noise identification (ARW, bias instability)
%  6. End-to-end check: attitude drift    (raw vs. calibrated gyro)
%
%  Requirements: base MATLAB R2016b+ (or GNU Octave 7+). No toolboxes.
%  Run time: well under a minute on a laptop.
%  ========================================================================
clear; close all; clc;
thisDir = fileparts(mfilename('fullpath'));
addpath(fullfile(thisDir, 'src'));
outDir = fullfile(thisDir, 'results');
if ~exist(outDir, 'dir'), mkdir(outDir); end

rng(42);                          % reproducible results
P    = imu_truth();               % hidden "true" sensor errors
fs   = P.fs;
rmsv = @(x) sqrt(mean(x(:).^2));
d2r  = pi/180;

%% 1) Accelerometer - lab calibration with known orientations -------------
% Sensor is mounted on a 2-axis gimbal and held static in 24 known poses.
nPos  = 24;  nS = 5*fs;           % 5 s per pose, averaged
F_ref = zeros(3, nPos);  Y_acc = zeros(3, nPos);
for i = 1:nPos
    R = rand_rotation();                         % known from the fixture
    f = R.' * [0; 0; P.g];                       % specific force, sensor frame
    y = P.acc.K*f + P.acc.b + P.acc.nd*sqrt(fs)*randn(3, nS);
    F_ref(:, i) = f;
    Y_acc(:, i) = mean(y, 2);
end
[Ka_hat, ba_hat] = calib_linear_lsq(Y_acc, F_ref);

%% 2) Accelerometer - field calibration (orientations unknown) ------------
[Ta_hat, ba_field, infoField] = calib_accel_field(Y_acc, P.g);

% Validate both methods on 50 new, unseen orientations
nVal = 50;
[nRaw, nLab, nField, vRaw, vLab] = deal(zeros(1, nVal));
for i = 1:nVal
    R = rand_rotation();  f = R.'*[0; 0; P.g];
    y = mean(P.acc.K*f + P.acc.b + P.acc.nd*sqrt(fs)*randn(3, fs), 2);
    fLab   = Ka_hat \ (y - ba_hat);
    fField = Ta_hat * (y - ba_field);
    nRaw(i)   = norm(y)      - P.g;
    nLab(i)   = norm(fLab)   - P.g;
    nField(i) = norm(fField) - P.g;
    vRaw(i)   = norm(y - f);
    vLab(i)   = norm(fLab - f);
end

figure('Name', 'Accelerometer calibration', 'Position', [60 60 1000 380]);
subplot(1, 2, 1);
semilogy(1:nVal, abs(nRaw)*1e3, 'o-', 1:nVal, abs(nLab)*1e3, 's-', ...
         1:nVal, abs(nField)*1e3, '^-');
grid on; xlabel('Validation orientation #'); ylabel('||f| - g|  (mm/s^2)');
legend('Raw', 'Lab cal (known poses)', 'Field cal (|f| = g only)', 'Location', 'best');
title('Gravity-magnitude error on unseen poses');
subplot(1, 2, 2);
vals = [rmsv(vRaw) rmsv(vLab)]*1e3;
bar(vals);  set(gca, 'YScale', 'log');  ylim([1 2000]);
text(1:2, vals*1.3, compose('%.1f', vals), 'HorizontalAlignment', 'center');
set(gca, 'XTickLabel', {'Raw', 'Lab cal'});  grid on;
ylabel('RMS vector error (mm/s^2)');  title('Full 3-D specific-force error');
saveas(gcf, fullfile(outDir, 'fig1_accel_calibration.png'));

%% 3) Gyroscope - rate-table calibration at 25 degC -----------------------
rates = [30 60 120 250];                          % deg/s, both directions
W_ref = zeros(3, 1 + 3*2*numel(rates));  c = 1;   % first column: at rest
for ax = 1:3
    for r = [rates, -rates]
        c = c + 1;  W_ref(ax, c) = r;
    end
end
nS    = 10*fs;
Y_gyr = zeros(size(W_ref));
for i = 1:size(W_ref, 2)
    y = P.gyr.K*W_ref(:, i) ...
      + gyro_thermal_bias(P, 25*ones(1, nS), zeros(1, nS)) ...
      + gauss_markov(nS, fs, P.gyr.gm_sigma, P.gyr.gm_tau, 3) ...
      + P.gyr.arw*sqrt(fs)*randn(3, nS);
    Y_gyr(:, i) = mean(y, 2);
end
[Kg_hat, bg25_hat] = calib_linear_lsq(Y_gyr, W_ref);
errBefore = Y_gyr - W_ref;
errAfter  = Kg_hat \ (Y_gyr - bg25_hat) - W_ref;

figure('Name', 'Gyro rate table', 'Position', [80 80 800 360]);
semilogy(1:size(W_ref, 2), sqrt(sum(errBefore.^2)), 'o-', ...
         1:size(W_ref, 2), sqrt(sum(errAfter.^2)), 's-');
grid on; xlabel('Rate-table step #'); ylabel('|rate error| (deg/s)');
legend('Raw', 'Calibrated'); title('Gyro scale factor / misalignment / bias calibration');
saveas(gcf, fullfile(outDir, 'fig2_gyro_rate_table.png'));

%% 4) Gyro thermal drift: data-driven compensation ------------------------
fsT = 1;                                          % chamber data logged at 1 Hz
% Training run: full range, and deliberately VARIED ramp rates. The ramp
% rate drives the hysteresis, so the model can only learn it if training
% covers many rates (try a single 1.5 degC/min rate and watch the MLP fail).
[tA, TA] = make_temp_profile([25 -20 70 -20 70 -20 70 25], ...
                             [1.5 0.5 2.0 1.0 0.3 1.2 0.7], 15, fsT, 120);
% Test run: different ramp rate and longer dwells (never seen in training)
[tB, TB] = make_temp_profile([25 70 0 50 25],       0.8, 30, fsT, 120);
Dtr = sim_thermal_run(P, tA, TA, fsT);
Dte = sim_thermal_run(P, tB, TB, fsT);

% Model A: cubic polynomial in temperature (classic datasheet approach)
predA = zeros(size(Dte.y));
for ax = 1:3
    pp = polyfit(Dtr.Tm - 25, Dtr.y(ax, :), 3);
    predA(ax, :) = polyval(pp, Dte.Tm - 25);
end
% Model B: linear regression with physics-motivated features (T^k + dT/dt)
feat  = @(D) [ones(size(D.Tm)); D.Tm-25; (D.Tm-25).^2; (D.Tm-25).^3; D.rate];
predB = (Dtr.y / feat(Dtr)) * feat(Dte);
% Model C: small neural network on [T, dT/dt]
net   = mlp_train([Dtr.Tm; Dtr.rate], Dtr.y, 16, 4000, 0.01);
predC = mlp_predict(net, [Dte.Tm; Dte.rate]);
% Baseline: single bias value from the 25 degC rate-table calibration
pred0 = repmat(bg25_hat, 1, numel(tB));

% Score against the noise-free thermal bias (white noise / bias wander are
% not predictable from temperature, so they should not count as model error)
modelNames = {'No comp. (25 degC bias)', 'Poly(T), 3rd order', ...
              'Linear: T^k + dT/dt', 'MLP on [T, dT/dt]'};
preds  = {pred0, predA, predB, predC};
rmseT  = cellfun(@(pr) rmsv(pr - Dte.bTrue), preds);

figure('Name', 'Thermal drift', 'Position', [100 40 1100 700]);
subplot(2, 2, 1);
plot(tA/60, TA, tB/60, TB); grid on;
xlabel('Time (min)'); ylabel('Sensor temperature (degC)');
legend('Training run', 'Test run', 'Location', 'best'); title('Climate-chamber profiles');
subplot(2, 2, 2);
up = Dtr.rate > 0.2;  dn = Dtr.rate < -0.2;
plot(Dtr.Tm(up), Dtr.y(3, up), '.', Dtr.Tm(dn), Dtr.y(3, dn), '.', 'MarkerSize', 4);
grid on; xlabel('Temperature (degC)'); ylabel('z-gyro bias (deg/s)');
legend('Heating', 'Cooling', 'Location', 'best'); title('Training data: thermal hysteresis');
subplot(2, 2, 3);
plot(tB/60, Dte.bTrue(3, :), 'k', 'LineWidth', 2); hold on;
plot(tB/60, predA(3, :), tB/60, predB(3, :), tB/60, predC(3, :));
grid on; xlabel('Time (min)'); ylabel('z-gyro bias (deg/s)');
legend('True', 'Poly(T)', 'Linear features', 'MLP', 'Location', 'best');
title('Test run: predicted vs true bias');
subplot(2, 2, 4);
bar(rmseT*1e3); set(gca, 'YScale', 'log'); ylim([1 500]); grid on;
text(1:4, rmseT*1e3*1.4, compose('%.1f', rmseT*1e3), 'HorizontalAlignment', 'center');
set(gca, 'XTickLabel', {'None', 'Poly', 'Lin.feat', 'MLP'});
ylabel('RMSE (mdeg/s)'); title('Thermal compensation error (unseen profile)');
saveas(gcf, fullfile(outDir, 'fig3_thermal_drift.png'));

%% 5) Allan variance: identify the stochastic error model -----------------
nAV  = 2*3600*fs;                                  % 2 h static log, z-axis
wAV  = gauss_markov(nAV, fs, P.gyr.gm_sigma, P.gyr.gm_tau, 1) ...
     + P.gyr.arw*sqrt(fs)*randn(1, nAV);
[tau, adev] = allan_deviation(wAV, fs, 60);
arwEst = exp(interp1(log(tau), log(adev), 0));     % read at tau = 1 s
% Bias instability = first flat region (first local minimum) of the curve.
% A global min() would pick the noisy tail at long tau instead.
iMin = find(diff(adev) > 0, 1);
if isempty(iMin), [~, iMin] = min(adev); end
adevMin = adev(iMin);
biEst  = adevMin/0.664;                            % IEEE Std 952 convention

figure('Name', 'Allan deviation', 'Position', [120 120 700 450]);
loglog(tau, adev, 'o-', 'LineWidth', 1.2); hold on;
loglog(tau, P.gyr.arw./sqrt(tau), '--');
loglog(tau(iMin), adevMin, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
grid on; xlabel('Cluster time \tau (s)'); ylabel('Allan deviation (deg/s)');
legend('Simulated data', 'True ARW slope (-1/2)', 'Bias-instability floor', ...
       'Location', 'southwest');
title('Gyro noise identification');
saveas(gcf, fullfile(outDir, 'fig4_allan_deviation.png'));

%% 6) End-to-end: attitude drift during 2 minutes of motion at 45 degC ----
N     = 120*fs;  t = (0:N-1)/fs;  dt = 1/fs;
wTrue = [40*sin(2*pi*0.20*t); 30*sin(2*pi*0.13*t + 1); 50*sin(2*pi*0.07*t + 2)];
Tamb  = 45*ones(1, N);
y = P.gyr.K*wTrue + gyro_thermal_bias(P, Tamb, zeros(1, N)) ...
  + gauss_markov(N, fs, P.gyr.gm_sigma, P.gyr.gm_tau, 3) ...
  + P.gyr.arw*sqrt(fs)*randn(3, N);
wEst = {y, ...                                              % raw
        Kg_hat \ (y - bg25_hat), ...                        % rate-table cal only
        Kg_hat \ (y - mlp_predict(net, [Tamb; zeros(1, N)]))}; % + thermal MLP

Rtrue = eye(3);  Rest = {eye(3), eye(3), eye(3)};  attErr = zeros(3, N);
for k = 1:N
    Rtrue = Rtrue * axang2R(wTrue(:, k)*d2r*dt);
    for j = 1:3
        Rest{j} = Rest{j} * axang2R(wEst{j}(:, k)*d2r*dt);
        cang = (trace(Rtrue.'*Rest{j}) - 1)/2;
        attErr(j, k) = acos(max(-1, min(1, cang)))/d2r;
    end
end

figure('Name', 'Attitude drift', 'Position', [140 140 800 420]);
semilogy(t, attErr.'); grid on;
xlabel('Time (s)'); ylabel('Attitude error (deg)');
legend('Raw gyro', 'Rate-table calibration', 'Rate table + thermal MLP', ...
       'Location', 'southeast');
title('Gyro-only attitude integration at 45 degC');
saveas(gcf, fullfile(outDir, 'fig5_attitude_drift.png'));

%% Results summary --------------------------------------------------------
fprintf('\n===================== RESULTS SUMMARY =====================\n');
fprintf('ACCELEROMETER\n');
fprintf('  Lab LS:   max |K err| = %.2e   max |b err| = %.2e m/s^2\n', ...
        max(abs(Ka_hat(:) - P.acc.K(:))), max(abs(ba_hat - P.acc.b)));
fprintf('  Field LM: %d iterations, fit residual RMS = %.2e m/s^2\n', ...
        infoField.iterations, infoField.rms);
fprintf('  |f|-g RMS on unseen poses  raw / lab / field: %.1f / %.2f / %.2f mm/s^2\n', ...
        rmsv(nRaw)*1e3, rmsv(nLab)*1e3, rmsv(nField)*1e3);
fprintf('GYROSCOPE (rate table)\n');
fprintf('  max |K err| = %.2e   max |b err| = %.2e deg/s\n', ...
        max(abs(Kg_hat(:) - P.gyr.K(:))), max(abs(bg25_hat - P.gyr.c0)));
fprintf('THERMAL COMPENSATION (RMSE on unseen profile)\n');
for k = 1:numel(modelNames)
    fprintf('  %-26s %8.2f mdeg/s\n', modelNames{k}, rmseT(k)*1e3);
end
fprintf('ALLAN VARIANCE\n');
fprintf('  ARW: true %.3f, estimated %.3f deg/sqrt(h)\n', P.gyr.arw*60, arwEst*60);
fprintf('  Bias instability: %.2f deg/h at tau = %.0f s\n', biEst*3600, tau(iMin));
fprintf('ATTITUDE ERROR after %d s\n', N/fs);
fprintf('  raw %.2f deg | rate-table cal %.2f deg | + thermal MLP %.2f deg\n', attErr(:, end));
fprintf('Figures saved to: %s\n', outDir);

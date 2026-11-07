# MEMS IMU Calibration & System Identification (MATLAB)

A simulation study of how a low-cost 6-axis MEMS IMU (accelerometer + gyroscope) is calibrated and characterised, from classical least-squares calibration to a neural-network model of thermal drift.

The sensor errors are simulated from a known "true" model, so every estimate can be checked against ground truth. Everything is written in base MATLAB with no toolboxes: the least-squares and Levenberg-Marquardt solvers, the neural network and the Allan variance are all implemented from scratch.

## What it covers

| # | Step | Method | Identified |
|---|------|--------|------------|
| 1 | Accelerometer lab calibration | Multi-position test, linear least squares | Scale factor, misalignment, bias (12 parameters) |
| 2 | Accelerometer field calibration | Levenberg-Marquardt on the constraint \|f\| = g; orientations unknown | 9 parameters |
| 3 | Gyro rate-table calibration | Linear least squares | Scale factor, misalignment, bias |
| 4 | Gyro thermal-drift compensation | Cubic polynomial vs. linear features vs. small neural network | Bias vs. temperature, including heating/cooling hysteresis |
| 5 | Noise identification | Overlapping Allan deviation | Angle random walk, bias instability |
| 6 | End-to-end validation | Gyro-only attitude integration | Attitude drift: raw vs. calibrated |

## Sensor error model

Accelerometer: `y = K_a f + b_a + n`

Gyroscope: `y = K_g w + b(T, dT/dt) + b_GM + n`

- `K` combines scale-factor errors and axis misalignment.
- `b(T, dT/dt)` is a cubic temperature drift plus thermal hysteresis: the bias is higher while heating than while cooling.
- `b_GM` is a first-order Gauss-Markov bias wander and `n` is white noise (angle random walk).

All parameter values are in `src/imu_truth.m` and can be changed to test other sensor grades.

## Results

| Step | Before | After | Improvement |
|---|---|---|---|
| Accelerometer, gravity-norm error on unseen poses (RMS) | 193.2 mm/s² | 1.37 mm/s² (lab) / 1.65 mm/s² (field) | ~140× |
| Accelerometer, 3-D vector error (RMS) | 275.3 mm/s² | 2.4 mm/s² | ~115× |
| Gyro rate error on rate table | 0.3–6 deg/s | 0.003–0.013 deg/s | ~300× |
| Gyro thermal bias, unseen chamber profile (RMSE) | 130.3 mdeg/s | 26.2 (poly) / 9.7 (linear) / **2.8 (MLP)** mdeg/s | ~47× |
| Attitude error after 120 s at 45 °C | 21.85° | 2.50° (rate table) / **0.34°** (+ thermal MLP) | ~64× |

**Noise identification (Allan variance):** angle random walk estimated at 0.611 deg/√h (true value 0.600, error < 2 %). Bias instability is 8.2 deg/h at τ ≈ 81 s.

### Takeaways

- The fixture-free field calibration gets almost the same gravity-norm accuracy as the lab method. It can't recover the absolute axis orientation, though, since any rotation leaves |f| unchanged.
- A temperature-only polynomial can't model thermal hysteresis because it can't tell heating from cooling. Adding dT/dt as an input fixes most of the error, and the neural network captures the rest of the nonlinearity.
- The network only generalises because the training run uses varied ramp rates. Trained on a single ramp rate, it does worse than the simple models on the test run, so experiment design matters as much as the model.
- Calibrating the gyro bias at 25 °C isn't enough at 45 °C. Thermal compensation cuts the attitude drift by another ~7×. What remains comes from random noise and bias wander, which calibration can't remove; that needs sensor fusion, e.g. a Kalman filter.
- The calibrated gyro error on the rate table matches the noise averaged over each 10 s step, so that calibration is limited by noise, not by the method.

## Figures

**Accelerometer calibration:** gravity-magnitude error on unseen orientations (left) and full 3-D error before and after calibration (right).

![Accelerometer calibration](results/fig1_accel_calibration.png)

**Gyro rate-table calibration:** rate error at each table step before and after calibration.

![Gyro rate-table calibration](results/fig2_gyro_rate_table.png)

**Thermal drift:** chamber profiles, hysteresis in the training data, predictions on the unseen test run, and RMSE per model.

![Thermal drift compensation](results/fig3_thermal_drift.png)

**Allan deviation:** the −½ slope gives the angle random walk; the flat region marks the bias instability.

![Allan deviation](results/fig4_allan_deviation.png)

**Attitude drift:** gyro-only orientation error over 2 minutes of motion at 45 °C.

![Attitude drift](results/fig5_attitude_drift.png)

## How to run

1. Clone or download this repository.
2. Open MATLAB (R2016b or newer) and set the repository folder as the Current Folder.
3. Run:

```matlab
>> main_mems_calibration
```

It takes under a minute. The figures are saved to `results/` and a results summary is printed to the Command Window. A fixed random seed (`rng(42)`) makes the results reproducible.

## Project structure

```
main_mems_calibration.m       main script, runs all six steps
src/
  imu_truth.m                 ground-truth sensor error parameters
  gyro_thermal_bias.m         thermal bias and hysteresis model
  gauss_markov.m              bias-instability process
  calib_linear_lsq.m          12-parameter linear calibration
  calib_accel_field.m         fixture-free Levenberg-Marquardt calibration
  make_temp_profile.m         climate-chamber profile with thermal lag
  sim_thermal_run.m           static gyro inside the chamber
  mlp_train.m, mlp_predict.m  small neural network trained with Adam
  allan_deviation.m           overlapping Allan deviation
  rand_rotation.m, axang2R.m  rotation helpers
results/                      generated figures
```

## Limitations

- All data is simulated. A real sensor also has effects not modelled here, such as gyro g-sensitivity, accelerometer thermal drift and flicker noise.
- The bias instability is simulated as a Gauss-Markov process, so the Allan curve shows a bump rather than the flat floor typical of real MEMS gyros. The 0.664 factor used to read bias instability assumes flicker noise, so the value is an approximation here.
- The neural-network prediction is slightly noisy on constant-temperature holds, because the temperature reading and the estimated temperature rate are noisy. Filtering the inputs would reduce this.

## Next steps

- Repeat the pipeline with a real sensor (e.g. BMI270 or ICM-42688 on an ESP32) and compare against the simulation.
- Run the thermal model on the microcontroller itself.
- Fuse the calibrated IMU in an extended Kalman filter to bound the remaining drift.

## License

MIT

function D = sim_thermal_run(P, t, T, fsT)
%SIM_THERMAL_RUN  Static gyro inside a climate chamber.
%   Output is decimated to fsT (e.g. 1 Hz), so white noise is averaged.
%   Returns measured temperature, estimated temperature rate, gyro output
%   and the noise-free thermal bias (for scoring only).
N         = numel(T);
dTdtTrue  = gradient(T)*fsT*60;                          % degC/min
D.bTrue   = gyro_thermal_bias(P, T, dTdtTrue);
D.y       = D.bTrue ...
          + gauss_markov(N, fsT, P.gyr.gm_sigma, P.gyr.gm_tau, 3) ...
          + P.gyr.arw*sqrt(fsT)*randn(3, N);             % static => output = bias
D.t       = t;
D.Tm      = T + 0.05*randn(size(T));                     % on-chip temp sensor
D.rate    = gradient(movmean(D.Tm, 61))*fsT*60;          % estimated degC/min
end

function b = gauss_markov(N, fs, sigma, tau, nAxes)
%GAUSS_MARKOV  First-order Gauss-Markov process (bias instability model).
%   Returns nAxes x N samples with steady-state std SIGMA and correlation
%   time TAU (s), started from its stationary distribution.
a  = exp(-1/(fs*tau));
w  = sigma*sqrt(1 - a^2) * randn(N, nAxes);
zi = a*sigma*randn(1, nAxes);
b  = filter(1, [1 -a], w, zi).';
end

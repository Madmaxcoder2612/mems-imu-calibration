function net = mlp_train(X, Y, nHidden, nEpochs, lr)
%MLP_TRAIN  Small neural network (1 tanh hidden layer), full-batch Adam.
%   X : nIn x N inputs,  Y : nOut x N targets.
%   Written from scratch so no Deep Learning Toolbox is needed.
nIn  = size(X, 1);  nOut = size(Y, 1);
net.xm = mean(X, 2);  net.xs = std(X, 0, 2);
net.ym = mean(Y, 2);  net.ys = std(Y, 0, 2);
Xn = (X - net.xm)./net.xs;
Yn = (Y - net.ym)./net.ys;

p = {randn(nHidden, nIn)*sqrt(1/nIn), zeros(nHidden, 1), ...
     randn(nOut, nHidden)*sqrt(1/nHidden), zeros(nOut, 1)};
m = cellfun(@(q) zeros(size(q)), p, 'UniformOutput', false);  v = m;
b1 = 0.9;  b2 = 0.999;  epsAdam = 1e-8;
net.loss = zeros(nEpochs, 1);

for ep = 1:nEpochs
    A1   = tanh(p{1}*Xn + p{2});                 % forward
    E    = p{3}*A1 + p{4} - Yn;
    net.loss(ep) = mean(E(:).^2);
    dY   = 2*E/numel(E);                         % backward
    dZ1  = (p{3}.'*dY).*(1 - A1.^2);
    grad = {dZ1*Xn.', sum(dZ1, 2), dY*A1.', sum(dY, 2)};
    for k = 1:4                                  % Adam update
        m{k} = b1*m{k} + (1 - b1)*grad{k};
        v{k} = b2*v{k} + (1 - b2)*grad{k}.^2;
        p{k} = p{k} - lr*(m{k}/(1 - b1^ep))./(sqrt(v{k}/(1 - b2^ep)) + epsAdam);
    end
end
net.params = p;
end

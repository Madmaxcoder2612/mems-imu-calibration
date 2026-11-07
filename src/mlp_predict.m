function Y = mlp_predict(net, X)
%MLP_PREDICT  Forward pass of a network trained with MLP_TRAIN.
p  = net.params;
Xn = (X - net.xm)./net.xs;
Y  = (p{3}*tanh(p{1}*Xn + p{2}) + p{4}).*net.ys + net.ym;
end

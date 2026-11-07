function S = skew3(v)
%SKEW3  Cross-product matrix: skew3(a)*b = cross(a, b).
S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

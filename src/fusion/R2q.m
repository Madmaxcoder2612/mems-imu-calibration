function q = R2q(R)
%R2Q  Unit quaternion [w; x; y; z] from a rotation matrix (Shepperd's method).
tr = trace(R);
if tr > 0
    s = sqrt(tr + 1)*2;
    q = [0.25*s; (R(3,2)-R(2,3))/s; (R(1,3)-R(3,1))/s; (R(2,1)-R(1,2))/s];
else
    [~, i] = max(diag(R));
    switch i
        case 1
            s = sqrt(1 + R(1,1) - R(2,2) - R(3,3))*2;
            q = [(R(3,2)-R(2,3))/s; 0.25*s; (R(1,2)+R(2,1))/s; (R(1,3)+R(3,1))/s];
        case 2
            s = sqrt(1 + R(2,2) - R(1,1) - R(3,3))*2;
            q = [(R(1,3)-R(3,1))/s; (R(1,2)+R(2,1))/s; 0.25*s; (R(2,3)+R(3,2))/s];
        otherwise
            s = sqrt(1 + R(3,3) - R(1,1) - R(2,2))*2;
            q = [(R(2,1)-R(1,2))/s; (R(1,3)+R(3,1))/s; (R(2,3)+R(3,2))/s; 0.25*s];
    end
end
q = q/norm(q);
if q(1) < 0, q = -q; end
end

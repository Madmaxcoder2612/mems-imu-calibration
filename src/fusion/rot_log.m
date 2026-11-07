function v = rot_log(R)
%ROT_LOG  Rotation vector (rad) of a rotation matrix.
th = acos(max(-1, min(1, (trace(R) - 1)/2)));
v  = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]/2;
if th > 1e-9
    v = v*th/sin(th);
end
end

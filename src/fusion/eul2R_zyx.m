function R = eul2R_zyx(roll, pitch, yaw)
%EUL2R_ZYX  Rotation matrix (body -> nav) from ZYX Euler angles (rad).
cr = cos(roll);  sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw);   sy = sin(yaw);
R = [cy -sy 0; sy cy 0; 0 0 1] * [cp 0 sp; 0 1 0; -sp 0 cp] * [1 0 0; 0 cr -sr; 0 sr cr];
end

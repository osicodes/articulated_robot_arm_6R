function [a,o,n] = RPY(R)
% Takes rotation matrix and returns the roll, pitch, yaw angles.



a = atan2(R(2,1),R(1,1));
o = atan2(-R(3,1),(R(1,1)*cos(a)+R(2,1)*sin(a)));
n = atan2((-R(2,3)*cos(a) + R(1,3)*sin(a)),(R(2,2)*cos(a) - R(1,2)*sin(a)));

end
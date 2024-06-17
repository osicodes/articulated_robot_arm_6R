function T = r_EULER_POSE(pos)
% Takes a pose (x y z phi th psi) and returns the transformation matrix (with euler rotation
% matrix).
% Example Input:
% 
% clear; clc;
% pose = [10 0 25 0 pi 0];
% T = r_EULER_POSE(pose);
% 
% Output:
% T =
% 
%    -1.0000         0    0.0000   10.0000
%          0    1.0000         0         0
%    -0.0000         0   -1.0000   25.0000
%          0         0         0    1.0000

T = [cos(pos(4))*cos(pos(5))*cos(pos(6))-sin(pos(4))*sin(pos(6))   -cos(pos(4))*cos(pos(5))*sin(pos(6))-sin(pos(4))*cos(pos(6))   cos(pos(4))*sin(pos(5))         pos(1);
     sin(pos(4))*cos(pos(5))*cos(pos(6))+cos(pos(4))*sin(pos(6))   -sin(pos(4))*cos(pos(5))*sin(pos(6))+cos(pos(4))*cos(pos(6))   sin(pos(4))*sin(pos(5))         pos(2);
     -sin(pos(5))*cos(pos(6))                             sin(pos(5))*sin(pos(6))                               cos(pos(5))                  pos(3);
     0                                             0                                              0                        1];
end
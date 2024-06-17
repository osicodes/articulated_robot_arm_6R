function [phi,th,psi] = EULER(R)
% Takes rotation matrix and returns the euler angles.
% Example Input:
% 
% clear; clc;
% R = [ 0.0000   -0.0000    1.0000
%      -0.0000   -1.0000         0
%       1.0000   -0.0000   -0.0000];
% [phi,th,psi] = EULER(R);
% 
% Output:
% phi = 0
% th = 1.5708
% psi = -3.1416
phi = atan2(R(2,3),R(1,3));
psi = atan2((-R(1,1)*sin(phi) + R(2,1)*cos(phi)),(-R(1,2)*sin(phi) + R(2,2)*cos(phi)));
th = atan2((R(1,3)*cos(phi) + R(2,3)*sin(phi)),R(3,3));
end
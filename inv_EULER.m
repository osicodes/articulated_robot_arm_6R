function [phi,th,psi] = inv_EULER(M)
% Takes rotation matrix and returns the euler angles.
% Example Input:
% 
% clear; clc;
% R = [ 0.0000   -0.0000    1.0000
%      -0.0000   -1.0000         0
%       1.0000   -0.0000   -0.0000];
% [phi,th,psi] = inv_EULER(R);
% 
% Output:
% phi = 0
% th = 1.5708
% psi = -3.1416
phi = atan2(M(2,3),M(1,3));
psi = atan2((-M(1,1)*sin(phi) + M(2,1)*cos(phi)),(-M(1,2)*sin(phi) + M(2,2)*cos(phi)));
th = atan2((M(1,3)*cos(phi) + M(2,3)*sin(phi)),M(3,3));
end
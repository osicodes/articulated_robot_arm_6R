function [x,y,z,phi,th,psi] = inv_EULER_T(M)
% Takes transformation matrix and returns the euler angles.
% Example Input:
% 
% clear; clc;
% T = [ 0.0000   -0.0000    1.0000 187.8553
%      -0.0000   -1.0000         0        0
%       1.0000   -0.0000   -0.0000 155.3553
%            0         0         0   1.0000];
% [x,y,z,phi,th,psi] = inv_EULER(T);
% 
% Output:
% phi = 0
% th = 1.5708
% psi = -3.1416
x = M(1,4);
y = M(2,4);
z = M(3,4);
phi = atan2(M(2,3),M(1,3));
psi = atan2((-M(1,1)*sin(phi) + M(2,1)*cos(phi)),(-M(1,2)*sin(phi) + M(2,2)*cos(phi)));
th = atan2((M(1,3)*cos(phi) + M(2,3)*sin(phi)),M(3,3));
end
function q = EULERzyz2QUAT(OR)
% Takes the euler angles and returns the rotation matrix.
% Reference: 
% Euler angles, quaternions, and transformation matrices
% (https://ntrs.nasa.gov/citations/19770024290)
% 
% Cayley’s method
% Example Input:
% 
% clear; clc;
% OR = [pi, 0, 0]
% q = EULERzyz2QUAT(OR)
% 
% Output:
% q = [0 0 0 1]
% 
phi = OR(1);
th = OR(2);
psi = OR(3);

q(1) = cos(1/2*th) * cos(1/2*(phi+psi));
q(2) = -sin(1/2*th) * sin(1/2*(phi-psi));
q(3) = sin(1/2*th) * cos(1/2*(phi-psi));
q(4) = cos(1/2*th) * sin(1/2*(phi+psi));
end
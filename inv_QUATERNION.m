function q = inv_QUATERNION(R)
% Takes the euler angles and returns the rotation matrix.
% Reference: 
% [1] A Survey on the Computation of Quaternions from Rotation Matrices
% (http://www.iri.upc.edu/files/scidoc/2083-A-Survey-on-the-Computation-
% of-Quaternions-from-Rotation-Matrices.pdf)
% [2] Approaching dual quaternions from matrix algebra
% (F. Thomas, “Approaching dual quaternions from matrix algebra,”
% IEEE Transactions on Robotics, Vol. 30, No. 5, pp. 1037-1048, 2014.)
% 
% 
% Example Input:
% 
% clear; clc;
% R = [1.0000    0.0000    0.0000
%     0.0000   -1.0000         0
%     0.0000    0.0000   -1.0000]
% q = inv_QUATERNION(R)
% 
% Output:
% q = []
% 

% ---------------Cayley’s method----------------
q(1) = 1/4 * sqrt((R(1,1)+R(2,2)+R(3,3)+1)^2 + (R(3,2)-R(2,3))^2 + (R(1,3)-R(3,1))^2 + (R(2,1)-R(1,2))^2);
q(2) = 1/4 * sqrt((R(3,2)-R(2,3))^2 + (R(1,1)-R(2,2)-R(3,3)+1)^2 +(R(2,1)+R(1,2))^2 + (R(1,3)+R(3,1))^2);
q(3) = 1/4 * sqrt((R(1,3)-R(3,1))^2 +(R(2,1)+R(1,2))^2 + (-R(1,1)+R(2,2)-R(3,3)+1)^2 +(R(3,2)+R(2,3))^2);
q(4) = 1/4 * sqrt((R(2,1)-R(1,2))^2 + (R(1,3)+R(3,1))^2 +(R(3,2)+R(2,3))^2 + (-R(1,1)-R(2,2)+R(3,3)+1)^2);
% ----------------------------------------------


% -------Shepperd’s method(e1)------------------
% t = R(1,1) + R(2,2) + R(3,3);
% r = sqrt(1+t);
% s = 1\(2*r);
% 
% q(1) = 1/2*r;
% q(2) = (R(3,2)-R(2,3))*s;
% q(3) = (R(1,3)-R(3,1))*s;
% q(4) = (R(2,1)-R(1,2))*s;
% -------------------------------------------------
end
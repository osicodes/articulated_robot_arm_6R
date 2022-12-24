function R = r_ANGLE_AXIS(r,th)
% Takes the unit vector axis (r) and rotation angle (th) around r.
% Example Input:
% 
% clear; clc;
% R = r_ANGLE_AXIS([1;-1;1],2/3*pi)
% 
% Output:
% R =
% 
%     1.0000   -2.3660    0.6340
%    -0.6340    1.0000   -2.3660
%     2.3660   -0.6340    1.0000

ct = 1 - cos(th);
R = [r(1)^2*ct+cos(th)           r(1)*r(2)*ct-r(3)*sin(th)   r(1)*r(3)*ct+r(2)*sin(th);
     r(1)*r(2)*ct+r(3)*sin(th)   r(2)^2*ct+cos(th)           r(2)*r(3)*ct-r(1)*sin(th);
     r(1)*r(3)*ct-r(2)*sin(th)   r(2)*r(3)*ct+r(1)*sin(th)   r(3)^2*ct+cos(th)];
end
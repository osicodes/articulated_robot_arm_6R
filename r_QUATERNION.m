function R = r_QUATERNION(q)
% Takes the euler angles and returns the rotation matrix.
% Source: en.wikipedia.org/wiki/Rotation_matrix#Quaternion
% Example Input:
% 
% clear; clc;
% q = [0 0.5 0.2 0.3]
% R = r_QUATERNION(q)
% 
% Output:
% R =
% 
%     1.0000    0.0000    0.0000
%     0.0000   -1.0000         0
%     0.0000    0.0000   -1.0000

R = [q(1)^2+q(2)^2-q(3)^2-q(4)^2   2*(q(2)*q(3)-q(1)*q(4))      2*(q(2)*q(4)+q(1)*q(3));
     2*(q(2)*q(3)+q(1)*q(4))       q(1)^2-q(2)^2+q(3)^2-q(4)^2  2*(q(3)*q(4)-q(1)*q(2));
     2*(q(2)*q(4)-q(1)*q(3))       2*(q(3)*q(4)+q(1)*q(2))      q(1)^2-q(2)^2-q(3)^2+q(4)^2];
 

%--------------------------------------------------
% A more efficient calculation in which the quaternion does 
% not need to be unit normalized is given below.
%-----------------------------------------------------------

% n = q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2;
% 
% if (n == 0)
%     s = 0;
% else
%     s = 2/n;
% end
% 
% R = [1-s*(q(3)^2+q(4)^2)       s*(q(2)*q(3)-q(1)*q(4))   s*(q(2)*q(4)+q(1)*q(3));
%      s*(q(2)*q(3)+q(1)*q(4))   1-s*(q(2)^2+q(4)^2)       s*(q(3)*q(4)-q(1)*q(2));
%      s*(q(2)*q(4)-q(1)*q(3))   s*(q(3)*q(4)+q(1)*q(2))   1-s*(q(2)^2+q(3)^2)];
%  
%  ----------------------------------------------------
end
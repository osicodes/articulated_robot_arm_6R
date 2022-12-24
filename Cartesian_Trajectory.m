function traj = Cartesian_Trajectory(Xstart, Xend, Tf, N, method)
% *** CARTESIAN TRAJECTORY GENERATION ***
% Reference: Robotics Modelling Planning (B. Sicilliano)
% Takes Xstart: The initial end-effector configuration,
%       Xend: The final end-effector configuration,
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
% Returns traj: The discretized trajectory as a list of N matrices in SE(3)
%               separated in time by Tf/(N-1). The first in the list is 
%               Xstart and the Nth is Xend .
% This function is similar to ScrewTrajectory, except the origin of the 
% end-effector frame follows a straight line, decoupled from the rotational
% motion.
% Example Input:
% 
% clear; clc;
% Xstart = [[1, 0, 0, 1]; [0, 1, 0, 0]; [0, 0, 1, 1]; [0, 0, 0, 1]];
% Xend = [[0, 0, 1, 0.1]; [1, 0, 0, 0]; [0, 1, 0, 4.1]; [0, 0, 0, 1]];
% Tf = 5;
% N = 4;
% method = 5;
% traj = Cartesian_Trajectory(Xstart, Xend, Tf, N, method)
% 
% Output:
% traj =
%    [  1,  0,  0,  1; 
%       0,  1,  0,  0;
%       0,  0,  1,  1;
%       0,  0,  0,  1 ]
%
% [  0.8951, -0.3063,  0.3237,  0.8111;
%    0.3237,  0.9461,       0,       0;
%   -0.3063,  0.1048,  0.9461,  1.6506;
%         0,       0,       0,       1]
%
% [  0.1048, -0.3063,  0.9461,  0.2888;
%    0.9461,  0.3237,       0,       0;
%   -0.3063,  0.8951,  0.3237,  3.4493;
%         0,       0,       0,       1]
%
%   [  0,  0,  1,  0.1;
%      1,  0,  0,    0;
%      0,  1,  0,  4.1;
%      0,  0,  0,    1]

timegap = Tf / (N - 1);
traj = cell(1, N);

[Rstart, pstart] = TransToRp(Xstart);
[Rend, pend] = TransToRp(Xend);

[Rs_phi,Rs_th,Rs_psi] = inv_EULER(Rstart); %ZYZ orientation
[Re_phi,Re_th,Re_psi] = inv_EULER(Rend);%ZYZ orientation

Rs = [Rs_phi,Rs_th,Rs_psi]';%ZYZ orientation
Re = [Re_phi,Re_th,Re_psi]';%ZYZ orientation
for i = 1: N
    if method == 1
        s = LinearTimeScaling(Tf,timegap * (i - 1));
    elseif method == 3
        s = CubicTimeScaling(Tf,timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf,timegap * (i - 1));
    end
    
    %----------ZYZ orientation----------
%     Or = Rs + s * (Re - Rs);
% 
%     traj{i} ...
%     = [r_EULER(Or(1),Or(2),Or(3)), ...
%        pstart + s * (pend - pstart); 0, 0, 0, 1];
   %-----------------------------------
   
   %----------Angle_axis orientation----------
   Rse = Rstart' * Rend;
   angle = acos((Rse(1,1)+Rse(2,2)+Rse(3,3)-1)/2.0);
   if (sin(angle) ~= 0)
       axis_r = 1/(2*sin(angle))*[Rse(3,2)-Rse(2,3);Rse(1,3)-Rse(3,1);Rse(2,1)-Rse(1,2)];
   else
       X = fprintf('Sine theta at the denominator is zero');
       disp(X);
       exit
   end
   Rot_r_th = r_ANGLE_AXIS(axis_r,s*angle);
   
   traj{i} ...
    = [Rstart*Rot_r_th, ...
       pstart + s * (pend - pstart); 0, 0, 0, 1];
   %------------------------------------------
end
end
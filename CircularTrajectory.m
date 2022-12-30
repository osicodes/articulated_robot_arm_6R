function traj = CircularTrajectory(Xcir, Ycir, Tf, N, r, Rstart, Rend)
% *** CHAPTER 9: TRAJECTORY GENERATION ***
% Takes Xcir: The center of circle on x-axis,
%       Ycir: The center of circle on y-axis,
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       r: The raduis of circle,
%       Rstart: The starting orientation,
%       Rend: The ending orientation,
% Returns traj: The discretized trajectory as a list of N matrices in SE(3)
%               separated in time by Tf/(N-1). The first in the list is 
%               Xstart with position at (r*cos(0)+Xcir,r*sin(0)+Ycir) and 
%               the Nth is Xend with position at (r*cos(2*pi)+Xcir,r*sin(2*pi)+Ycir).
% Example Input:
% 
% clear; clc;
% Xcir = ;
% Ycir = ;
% Tf = 5;
% N = 4;
% r = 5;
% traj = CircularTrajectory(Xcir, Ycir, Tf, N, r)
% 
% Output:
% traj =
%    1.0000         0         0    1.0000
%         0    1.0000         0         0
%         0         0    1.0000    1.0000
%         0         0         0    1.0000
%
%    0.9366   -0.2140    0.2774    0.8111
%    0.2774    0.9366   -0.2140         0
%   -0.2140    0.2774    0.9366    1.6506
%         0         0         0    1.0000
%
%    0.2774   -0.2140    0.9366    0.2889
%    0.9366    0.2774   -0.2140         0
%   -0.2140    0.9366    0.2774    3.4494
%         0         0         0    1.0000
%
%   -0.0000    0.0000    1.0000    0.1000
%    1.0000   -0.0000    0.0000         0
%    0.0000    1.0000   -0.0000    4.1000
%         0         0         0    1.0000

timegap = Tf / (N - 1);
traj = cell(1, N);

[Rs_phi,Rs_th,Rs_psi] = inv_EULER(Rstart); %ZYZ orientation
[Re_phi,Re_th,Re_psi] = inv_EULER(Rend);%ZYZ orientation

Rs = [Rs_phi,Rs_th,Rs_psi]';%ZYZ orientation
Re = [Re_phi,Re_th,Re_psi]';%ZYZ orientation

for i = 1: N
    s = CircularTimeScaling(Tf,timegap * (i - 1));
    
%     Or = Rs + s * (Re - Rs);
    
    center = [Xcir;Ycir;100];
    p = [r*cos(2*pi*s);r*sin(2*pi*s);0];
    ra = [1 0 0;0 1 0;0 0 1];

%     traj{i} ...
%     = [r_EULER(Or(1),Or(2),Or(3)), ...
%        center + ra*p; 0, 0, 0, 1];
   
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
       center + ra*p; 0, 0, 0, 1];
   %------------------------------------------
end
end
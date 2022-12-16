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
for i = 1: N
    s = CircularTimeScaling(Tf,timegap * (i - 1));
    
    traj{i} ...
    = [Rstart * MatrixExp3(MatrixLog3(Rstart' * Rend) * s), ...
       [r*cos(2*pi*s)+Xcir;r*sin(2*pi*s)+Ycir;100]; 0, 0, 0, 1];
end
end
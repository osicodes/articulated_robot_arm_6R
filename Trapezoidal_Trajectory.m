function traj = Trapezoidal_Trajectory(thetastart, thetaend, vmax, amax, eucPathLength, N)
% *** Trapezoidal TRAJECTORY GENERATION ***
% Reference: Robotics Modelling Planning (B. Sicilliano)
%   vmax: The maximum velocity.
%   amax: The maximum acceleration.
%   eucPathLength: The Eucledian path length.
%   t: The current time t satisfying 0 < t < Tf.
%   Tf: Total time of the motion in seconds from rest to rest = (eucPathLength*amax+vmax^2)/(amax*vmax),
% Returns s: The path parameter s(t) corresponding to trapezoidal time scale.
% Example Input:
% 
% clear; clc;
% t = 0.4;
% vmax = 2;
% amax = 0.6;
% eucPathLength = 6;
% s = TrapezoidalTimeScaling(t, vmax, amax, eucPathLength)
% 
% Output:
% s =
%    0.0080

Tf = (eucPathLength*amax+vmax^2)/(amax*vmax);
timegap = Tf / (N - 1);
traj = zeros(size(thetastart, 1), N);
for i = 1: N
    t = timegap * (i - 1);
    % acceleration
    if (t>=0.0 && t<=vmax/amax)
        s = amax*t*t/2.0;
    end

    % constant maximum speed
    if (t>vmax/amax && t<=eucPathLength/vmax)
        s = vmax*t - (vmax^2)/(2.0*amax);
    end
    
    % deceleration 
    if (t>eucPathLength/vmax)
        s = vmax*Tf - vmax^2/amax - amax*(t-Tf)^2/2.0;
    end
    traj(:, i) = thetastart + s/eucPathLength * (thetaend - thetastart);
end
traj = traj';
end
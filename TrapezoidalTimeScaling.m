function s = TrapezoidalTimeScaling(t, vmax, amax, eucPathLength)
% *** Trapezoidal TRAJECTORY GENERATION ***
% Reference: Robotics Modelling Planning (B. Sicilliano)
%   vmax: The maximum velocity.
%   amax: The maximum acceleration.
%   eucPathLength: The Eucledian path length.
%   t: The current time t satisfying 0 < t < Tf.
%   Total time  = (eucPathLength*amax+vmax^2)/(amax*vmax),
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

% acceleration
if (t>=0.0 && t<=vmax/amax)
  sigma = amax*t*t/2.0;
end

% constant maximum speed
if (t>vmax/amax && t<=eucPathLength/vmax)
  sigma = vmax*t - (vmax^2)/(2.0*amax);
end
    
% deceleration 
if (t>eucPathLength/vmax)
  sigma = vmax*Tf - vmax^2/amax - amax*(t-Tf)^2/2.0;
end
    
s = sigma/eucPathLength;
end
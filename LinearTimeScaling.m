function s = LinearTimeScaling(Tf, t)
% *** CHAPTER 9: TRAJECTORY GENERATION ***
% Takes Tf: Total time of the motion in seconds from rest to rest,
%       t: The current time t satisfying 0 < t < Tf.
% Returns s: The path parameter s(t) corresponding to linear time scale.
% Example Input: 
% 
% clear; clc;
% Tf = 2;
% t = 0.6;
% s = LinearTimeScaling(Tf,t)
% 
% Output:
% s =
%    0.2160

s = t / Tf;

% s =10 * (t / Tf)^3 - 15 * (t / Tf)^4 + 6 * (t / Tf)^5; %3-4-5 trajectory


end
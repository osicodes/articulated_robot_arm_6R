function s = CircularTimeScaling(Tf, t)
% *** CHAPTER 9: TRAJECTORY GENERATION ***
% Takes Tf: Total time of the motion in seconds from rest to rest,
%       t: The current time t satisfying 0 < t < Tf.
% Returns s: The path parameter s(t) corresponding to a third-order 
%            polynomial motion that begins and ends at zero velocity.
% Example Input: 
% 
% clear; clc;
% Tf = 2;
% t = 0.6;
% s = CircularTimeScaling(Tf,t)
% 
% Output:
% s =
%    0.3000

s = t / Tf;
end
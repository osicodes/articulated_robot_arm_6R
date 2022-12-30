function traj = Cartesian_Trajectory(Xstart, Xend, Tf, N, method, varargin)
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

if nargin > 5
    if numel(varargin) == 3
        vmax = varargin{1};
        amax = varargin{2};
        eucPathLength = varargin{3};
    else
        msg = 'Input the velocity, acceleration and euclidean length between points';
        error(msg);
    end
end


timegap = Tf / (N - 1);
traj = cell(1, N);

[Rstart, pstart] = TransToRp(Xstart);
[Rend, pend] = TransToRp(Xend);


for i = 1: N
    switch(method)
        case 1
            s = LinearTimeScaling(Tf,timegap * (i - 1));
        case 2
            if numel(varargin) == 3
                Totaltime = (eucPathLength*amax+vmax^2)/(amax*vmax);
                deltaTime = Totaltime / (N - 1);
                s = TrapezoidalTimeScaling(deltaTime * (i - 1), vmax, amax, eucPathLength);
            else
                msg = 'Input the velocity, acceleration and euclidean length between points for method = 2';
                error(msg);
            end
        case 3
            s = CubicTimeScaling(Tf, timegap * (i - 1));
        case 5
            s = QuinticTimeScaling(Tf, timegap * (i - 1));
        otherwise
            error('Selected method should be 1, 2, 3 or 5');
    end
    
%----------ZYZ orientation----------
    [Rs_phi,Rs_th,Rs_psi] = inv_EULER(Rstart); %ZYZ orientation
    [Re_phi,Re_th,Re_psi] = inv_EULER(Rend);%ZYZ orientation
   
    Rs = [Rs_phi,Rs_th,Rs_psi]';%ZYZ orientation
    Re = [Re_phi,Re_th,Re_psi]';%ZYZ orientation
    Or = Rs + s * (Re - Rs);

    traj{i} ...
    = [r_EULER(Or(1),Or(2),Or(3)), ...
       pstart + s * (pend - pstart); 0, 0, 0, 1];
%-----------------------------------
   
%----------Angle_axis orientation----------
% 
%    Rse = Rstart' * Rend;
%    angle = acos((Rse(1,1)+Rse(2,2)+Rse(3,3)-1)/2.0);
%    if (sin(angle) ~= 0)
%        axis_r = 1/(2*sin(angle))*[Rse(3,2)-Rse(2,3);Rse(1,3)-Rse(3,1);Rse(2,1)-Rse(1,2)];
%    else
%        X = fprintf('Sine theta at the denominator is zero');
%        disp(X);
%        exit
%    end
%    Rot_r_th = r_ANGLE_AXIS(axis_r,s*angle);
%    
%    traj{i} ...
%     = [Rstart*Rot_r_th, ...
%        pstart + s * (pend - pstart); 0, 0, 0, 1];
%------------------------------------------
   
%-----------------SLERP--------------------
% % Quaternions give a simple way to encode this 
% % axis–angle representation in four numbers
% % Reference: Ken Shoemake. Animating rotation with quaternion curves. 
% % SIGGRAPH Computer Graphics, 19(3):245–254, 1985. doi:10.1145/325165.325242.
% % Sources: https://splines.readthedocs.io/en/latest/rotation/slerp.html
% 
% %.................Using created functions
% Rstart = validateRotMatrix(Rstart);% check if matrix is rotation matrix
% Rend = validateRotMatrix(Rend);
%    
% qs = inv_QUATERNION(Rstart);% from rotation matrix to quaternion
% qe = inv_QUATERNION(Rend);
%    
% qs = normalizeQUAT(qs);%normalize quaternion
% qe = normalizeQUAT(qe);
%    
% dp = QuatDot(qs,qe);%dot product
% % Negative dot product, the quaternions aren't pointing the same way 
% % (one positive, one negative). Flip the second one.
% if dp < 0
%     qe = -1 * qe;
% end
%   
% %    Use either glenn davis slerp or wiki slerp
% %    +++++++ Glenn Davis Slerp ++++++++++
% theta0 = acos(dp);
% sinv = 1/sin(theta0);
% qnumerator = qs*sin((1- s)*theta0) + qe*sin(s*theta0);
% qt =  qnumerator* sinv;
% %    ++++++++++++++++++++++++++++++++++++
%    
% %    +++++ Wiki Slerp +++not in use++++++++++
% %    inv_qs = QuatInverse(qs);
% %    d = QuatMultiply(inv_qs,qe);
% %    post = exp(s*log(d));
% %    qt = QuatMultiply(qs,post);
% %    ++++++++++++++++++
% 
% %................
% 
% 
%    
% %...........Using MATLAB functions
% %    qqs =  normalize(quaternion(Rstart, 'rotmat', 'point'));
% %    qqe =  normalize(quaternion(Rend, 'rotmat', 'point'));
% %    qt = compact(slerp(qqs,qqe,s));
% %...........
% 
% traj{i} ...
%  = [r_QUATERNION(qt), ...
%     pstart + s * (pend - pstart); 0, 0, 0, 1];
%------------------------------------------
end
end
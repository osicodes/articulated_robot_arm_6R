%   Reference: 
%   [1] Liao Z, Jiang G, Zhao F, Mei X, Yue Y. A novel solution of 
%       inverse kinematic for 6R robot manipulator with offset joint based 
%       on screw theory. International Journal of Advanced Robotic Systems. 
%       2020;17(3). doi:10.1177/1729881420925645
%   [2] Algorithmic approach to geometric solution of generalized 
%       Paden–Kahan subproblem and its extension
%       https://journals.sagepub.com/doi/pdf/10.1177/1729881418755157

tip=42.5;

d1=75.36; a1=35.36; a2=80.04; d4=100; d6 = 10+tip;
syms th1 th2 th3 th4 th5 th6

pa = zeros(3,6);
pa(:,1) = [0 0 0];
pa(:,2) = [a1 0 d1];
pa(:,3) = [a1 0 d1+a2];
pa(:,4) = [a1+d4 0 d1+a2];
pa(:,5) = [a1+d4 0 d1+a2];
pa(:,6) = [a1+d4 0 d1+a2];

sw = zeros(3,6);
sw(:,1) = [0 0 1];
sw(:,2) = [0 -1 0];
sw(:,3) = [0 -1 0];
sw(:,4) = [1 0 0];
sw(:,5) = [0 -1 0];
sw(:,6) = [1 0 0];

sv = zeros(3,6);
for i = 1:6
    sv(:,i) = Vector3Cross(-1*sw(:,i),pa(:,i));
end

s = [sw;sv];

% qr = [0; 0; 0; 0; 0; 0];
qr = [pi/3 -pi/6 -pi/3 0 pi/6 0]';

g0 = [0 0 1 a1+d4+d6;
    0 -1 0 0;
    1 0 0 d1+a2;
    0 0 0 1];

gst = FKinSpace(g0, s, qr); % Target pose transformation

Texp1 = revoluteExpMatrix(VecTose3(s(:,1)),th1);
Texp2 = revoluteExpMatrix(VecTose3(s(:,2)),th2);
Texp3 = revoluteExpMatrix(VecTose3(s(:,3)),th3);
Texp4 = revoluteExpMatrix(VecTose3(s(:,4)),th4);
Texp5 = revoluteExpMatrix(VecTose3(s(:,5)),th5);
Texp6 = revoluteExpMatrix(VecTose3(s(:,6)),th6);

%% ------------INVERSE----------------

q1 = pa(:,1);
q2 = pa(:,2);
q3 = pa(:,3);
q4 = pa(:,4);
q6 = pa(:,6);

% Remember, q4, q5, q6 are same points.
% When theta1 is rotated, q6 will never change and rotate by theta1
% So to get theta1, move from q6 at home position to q6 at target position

home = q6;  % q6 at home position

P06=gst(1:3,4);
P05 = P06 - d6*gst(1:3,3); % q6 at target position

target = P05; 

th1p = home;
th1q = target; 
th1r = pa(:,1);
th1omega = sw(:,1);

th1U = th1p - th1r;
th1V = th1q - th1r;

% th1u_prime and th1v_prime are projections on the x-y plane
% with theta1 between both vectors
th1u_prime = th1U - th1omega*th1omega'*th1U;
th1v_prime = th1V - th1omega*th1omega'*th1V;

theta1 = atan2(th1omega'*Vector3Cross(th1u_prime,th1v_prime)',...
    th1u_prime'*th1v_prime);

theta1_d = theta1 * 180/pi

% --------------------------------
% theta2 and theta3
% Using subproblem 3
% --------------------------------

% ----------theta3----------------
% Since position of q2 is affected by theta1 (rotation about z),we
% apply rotation matrix Rot(z,theta1)
% Rot(z,theta1) = [cos(theta1) -sin(theta1) 0]
%                 [sin(theta1)  cos(theta1) 0]
%                 [0            0           1]
rot_z = [cos(theta1) -sin(theta1) 0;...
    sin(theta1)  cos(theta1) 0;...
    0            0           1]

th3p = rot_z*home;
th3q = rot_z*q2
th3r = rot_z*pa(:,3)
th3omega = rot_z*sw(:,3);

delta = norm(target - th3q);
delta_prime_square = delta^2 - norm(th3omega' * (th3p - th3q))^2;

th3U = th3p - th3r;
th3V = th3q - th3r;

th3u_prime = th3U - th3omega*th3omega'*th3U;
th3v_prime = th3V - th3omega*th3omega'*th3V;

theta_not = atan2(th3omega'*Vector3Cross(th3u_prime,th3v_prime)',...
    th3u_prime'*th3v_prime);

phi_num = round(norm(th3U)^2 + norm(th3V)^2 - delta_prime_square, 4);
phi_den = round(2 * norm(th3U) * norm(th3V), 4);
phi = acos(phi_num/phi_den);

theta3 = (theta_not + phi)*180/pi
% theta3 = theta_not - phi


%------------theta2---------------

% Since q4, q5, q6 have same pose, get the orientation of joint 4
% and multiply the column_z-axis orientation with d4, then subtract from
% P05 to get actual position of P03
% --------------
point = zeros(3,6);
point(:,1) = [0 0 0];
point(:,2) = [a1 0 d1];
point(:,3) = [a1 0 d1+a2];
point(:,4) = [a1+d4 0 d1+a2];

sw_axis = zeros(3,6);
sw_axis(:,1) = [0 0 1];
sw_axis(:,2) = [0 -1 0];
sw_axis(:,3) = [0 -1 0];
sw_axis(:,4) = [1 0 0];

sv_axis = zeros(3,6);
for i = 1:6
    sv_axis(:,i) = Vector3Cross(-1*sw_axis(:,i),point(:,i));
end

s_axis = [sw_axis;sv_axis];

joints = qr(1:4);

g04 = [0 0 1 a1+d4;
    0 -1 0 0;
    1 0 0 d1+a2;
    0 0 0 1];

gst04 = FKinSpace(g04, s_axis, joints) 
% --------------
P03 = P05 - d4*gst04(1:3,3);

% q7_bar from the article
q7_bar = target; % i.e. P05

th2p = rot_z*q3; 
th2q = P03;  
th2r = rot_z*pa(:,2);
th2omega = rot_z*sw(:,2);

th2U = th2p - th2r;
th2V = th2q - th2r;

% th1u_prime and th1v_prime are projections on the x-y plane
% with theta1 between both vectors
th2u_prime = th2U - th2omega*th2omega'*th2U;
th2v_prime = th2V - th2omega*th2omega'*th2V;

delta_prime = target - th3q

theta2 = round(atan2(th2omega'*Vector3Cross(th2u_prime,th2v_prime)',...
    th2u_prime'*th2v_prime)*180/pi, 4)

gst
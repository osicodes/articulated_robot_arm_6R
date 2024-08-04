
% d0 is the distance from the ground to the end effector when the end
% effector shaft is fully down
d0=63; a1=225; a2=175;
%pa = point on arbitrary screw axis
%sw = a unit vector in the direction of the screw axis

% prismatic joint d3 is between 0 and 85mm
% 0 -> when the end effector shaft is fully down
% 85 -> when the end effector shaft is fully up

pa = zeros(3,3);
sw = zeros(3,4);

%% Axis 1 - revolute
pa(:,1) = [0 0 0];
sw(:,1) = [0 0 1];

%% Axis 2 - revolute
pa(:,2) = [a1 0 0];
sw(:,2) = [0 0 1];

%% Axis 3 - revolute
pa(:,3) = [a1+a2 0 0];
sw(:,3) = [0 0 -1];

%% Axis 4 - prismatic
v4 = [0 0 1]; % v4 is a unit vector pointing in the direction of translation
sw(:,4) = [0 0 0];

%% sv is linear velocity
sv = zeros(3,4);
% Linear velocities for revolute joints 1 to 3
for i = 1:3
    sv(:,i) = Vector3Cross(-1*sw(:,i),pa(:,i));
end

% Linear velocities for prismatic joints 4
sv(:,4) = v4;

s = [sw;sv];

qr = [-pi/6; -pi/3; pi; 30];

g0 = [1  0  0 a1+a2;
     0 -1  0     0;
     0  0 -1    d0;
     0  0  0     1];

gst = FKinSpace(g0, s, qr)

%% INVERSE

p1 = pa(:,1);
p2 = pa(:,2);
p3 = pa(:,3);
% target = [gst(1:2,4);0];
exp4 = MatrixExp6(VecTose3(s(:,4) * d3));
g1 = gst * inv(g0) * inv(exp4);
target = g1 * [p3;1];


%---------d3---------------

d3 = gst(3,4) - d0;

%---------theta2-----------
% Apply subproblem 3
th2p = p3;
th2q = p1;
th2r = p2;
th2omega = sw(:,2);


delta = norm(target(1:3) - th2q);
delta_prime_square = delta^2 - norm(th2omega' * (th2p - th2q))^2;

th2U = th2p - th2r;
th2V = th2q - th2r;


th2u_prime = th2U - th2omega*th2omega'*th2U;
th2v_prime = th2V - th2omega*th2omega'*th2V;

theta_not = atan2(th2omega'*Vector3Cross(th2u_prime,th2v_prime)',...
    th2u_prime'*th2v_prime);
% theta_not = atan((th2omega'*Vector3Cross(th2u_prime,th2v_prime)')/(th2u_prime'*th2v_prime));
% theta_not * 180/pi


phi_num = round(norm(th2U)^2 + norm(th2V)^2 - delta_prime_square, 4);
phi_den = round(2 * norm(th2U) * norm(th2V), 4);
phi = acos(phi_num/phi_den);
% phi * 180/pi

P04=gst(1:3,4);
P02 = P04 - a2*gst(1:3,1);

if (target(1) > 0 && target(2) > 0)
    if (target(2) > P02(2))
        theta2 = (theta_not - phi);
    elseif (target(2) < P02(2))
        theta2 = (-1*theta_not + phi);
    end
elseif (target(1) > 0 && target(2) < 0)
    if (target(2) > P02(2))
        theta2 = (theta_not - phi);
    elseif (target(2) < P02(2))
        theta2 = (-1*theta_not + phi);
    end
end

theta2_d = theta2 *180/pi


%---------theta1-----------
% Apply subproblem 1
exp2 = MatrixExp6(VecTose3(s(:,2) * theta2));

th1p_T = exp2 * [p3;1];
th1p = th1p_T(1:3);
th1q = target(1:3);
th1r = p1;
th1omega = sw(:,1);

th1U = th1p - th1r;
th1V = th1q - th1r;

th1u_prime = th1U - th1omega*th1omega'*th1U;
th1v_prime = th1V - th1omega*th1omega'*th1V;

theta1 = round(atan((th1omega'*Vector3Cross(th1u_prime,th1v_prime)')/...
    (th1u_prime'*th1v_prime)),4);

theta1_d = theta1 * 180/pi


%---------theta3-----------
% Apply subproblem 1
exp1 = MatrixExp6(VecTose3(s(:,1) * theta1));

th3p_T = inv(exp2) * inv(exp1) * g1 * [p2;1];
th3p = th3p_T(1:3);
th3q = target(1:3);
th3r = p3;
th3omega = sw(:,3);

th3U = th3p - th3r;
th3V = th3q - th3r;

th3u_prime = th3U - th3omega*th3omega'*th3U;
th3v_prime = th3V - th3omega*th3omega'*th3V;

theta3 = round(atan((th3omega'*Vector3Cross(th3u_prime,th3v_prime)')/...
    (th3u_prime'*th3v_prime)),4);

theta3_d = theta3 * 180/pi

% -------validate-----

qr_new = [theta1; theta2; theta3; d3];

g0 = [1  0  0 a1+a2;
     0 -1  0     0;
     0  0 -1    d0;
     0  0  0     1];

gst_new = FKinSpace(g0, s, qr_new)

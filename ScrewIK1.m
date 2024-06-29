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
% qr = [theta1 theta2 theta3 theta4 theta5 theta6]';
qr = [0 pi/4 0 pi/2 pi/4 0]';

g0 = [0 0 1 a1+d4+d6;
    0 -1 0 0;
    1 0 0 d1+a2;
    0 0 0 1];

% gst = FKinBody(g0, s, qr) % Target pose transformation
gst = ScrewFK1(g0, s, qr)

% gst = [         0         0    1.0000  50.8600;
%          0   -1.0000         0         50;
%     1.0000         0         0  105.4000;
%          0         0         0    1.0000]


%% ------------INVERSE----------------

q1 = pa(:,1);
q2 = pa(:,2);
q3 = pa(:,3);
q4 = pa(:,4);
q6 = pa(:,6);

% Remember, q4, q5, q6 are same points.
% When theta1 is rotated, q6 will never change and rotate by theta1
% So to get theta1, move from q6 at home position to q6 at target position


P06=gst(1:3,4);
P05 = P06 - d6*gst(1:3,3); % q6 at target position

target = P05; 

th1p = q6;
th1q = target; 
th1r = pa(:,1);
th1omega = sw(:,1);

th1U = th1p - th1r;
th1V = th1q - th1r;

% th1u_prime and th1v_prime are projections on the x-y plane
% with theta1 between both vectors
th1u_prime = th1U - th1omega*th1omega'*th1U;
th1v_prime = th1V - th1omega*th1omega'*th1V;

theta1 = atan((th1omega'*Vector3Cross(th1u_prime,th1v_prime)')/...
    (th1u_prime'*th1v_prime));

theta1_d = theta1 * 180/pi;

% --------------------------------
% theta2 and theta3
% Using subproblem 3 for theta3
% --------------------------------

% ----------theta3----------------
% Since position of q2 is affected by theta1 (rotation about z),we
% apply rotation matrix Rot(z,theta1)
% Rot(z,theta1) = [cos(theta1) -sin(theta1) 0]
%                 [sin(theta1)  cos(theta1) 0]
%                 [0            0           1]
rot_z = [cos(theta1) -sin(theta1) 0;...
    sin(theta1)  cos(theta1) 0;...
    0            0           1];

th3p = rot_z*q6;
th3q = rot_z*q2;
th3r = rot_z*pa(:,3);
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

theta3 = (theta_not + phi);

theta3_d = theta3 *180/pi;
% theta3 = theta_not - phi


%------------theta2---------------

% Since q4, q5, q6 have same pose
% --------------
exp3 = MatrixExp6(VecTose3(s(:, 3) * theta3));

% q7_bar from the article
q7_bar = exp3 * [q4; 1];

q3_bar = target; % i.e. P05

th2p = rot_z*q7_bar(1:3); 
th2q = q3_bar;  
th2r = rot_z*pa(:,2);
th2omega = rot_z*sw(:,2);

th2U = th2p - th2r;
th2V = th2q - th2r;

% th1u_prime and th1v_prime are projections on the x-y plane
% with theta1 between both vectors
th2u_prime = th2U - th2omega*th2omega'*th2U;
th2v_prime = th2V - th2omega*th2omega'*th2V;


theta2 = round(atan2(th2omega'*Vector3Cross(th2u_prime,th2v_prime)',...
    th2u_prime'*th2v_prime), 4);

theta2_d = theta2*180/pi;



% ------------------------------------------
% theta4, 5 and 6
% ------------------------------------------

exp1 = MatrixExp6(VecTose3(s(:,1) * theta1));
exp2 = MatrixExp6(VecTose3(s(:,2) * theta2));
exp3 = MatrixExp6(VecTose3(s(:,3) * theta3));

T03 = exp1*exp2*exp3;


q6_T = [a1+d4+d6 0 d1+a2 1]';
q6_prime = T03 * q6_T;

r45_T = [pa(:,4); 1];
r45 = T03 * r45_T;

th4omega_T = [sw(:,4); 0];
th4omega_R = T03 * th4omega_T;

th5omega_T = [sw(:,5); 0];
th5omega_R = T03 * th5omega_T;

th45p = q6_prime(1:3);
th45q = gst(1:3,4);  
th45r = r45(1:3);
th4omega = th4omega_R(1:3);
th5omega = th5omega_R(1:3);

th45U = th45p - th45r;
th45V = th45q - th45r;

% To define z, define alpha, beta and gamma
alpha_num = (th4omega'*th5omega)*th4omega'*th45U - th4omega'*th45V;
alpha_den = (th4omega'*th5omega)^2 - 1;
alpha = round(alpha_num/alpha_den, 2);

beta_num = (th4omega'*th5omega)*th5omega'*th45V - th5omega'*th45U;
beta_den = (th4omega'*th5omega)^2 - 1;
beta = round(beta_num/beta_den, 2);

gamma_square_num = round(norm(th45U)^2 - alpha^2 - beta^2 - ...
    2*alpha*beta*th4omega'*th5omega, 4);
gamma_square_den = norm(Vector3Cross(th4omega,th5omega))^2;
gamma_square = gamma_square_num/gamma_square_den;

gamma = sqrt(gamma_square);

z = alpha * th4omega + beta * th5omega + gamma * ...
    Vector3Cross(th4omega,th5omega)';

% -----------theta5-------------
th45u_prime = th45U - th5omega*th5omega'*th45U;
z5_prime = z - th5omega*th5omega'*z;

theta5 = round(atan2(th5omega'*Vector3Cross(th45u_prime,z5_prime)',...
    th45u_prime'*z5_prime), 4);

theta5_d = theta5*180/pi;


% ------------theta4------------
z4_prime = z - th4omega*th4omega'*z;
th45v_prime = th45V - th4omega*th4omega'*th45V;

theta4 = round(atan2(th4omega'*Vector3Cross(z4_prime,th45v_prime)',...
    z4_prime'*th45v_prime), 4);

theta4_d = theta4*180/pi;

% ------------theta6--------------

th6p = gst(1:3,4);

exp1 = MatrixExp6(VecTose3(s(:, 1) * theta1));
exp2 = MatrixExp6(VecTose3(s(:, 2) * theta2));
exp3 = MatrixExp6(VecTose3(s(:, 3) * theta3));
exp4 = MatrixExp6(VecTose3(s(:, 4) * theta4));
exp5 = MatrixExp6(VecTose3(s(:, 5) * theta5));

T05 = exp1*exp2*exp3*exp4*exp5;

th6q =  inv(exp5) * inv(exp4) * inv(exp3) * ...
    inv(exp2) * inv(exp1) * gst *  inv(g0) *  [th6p; 1];

% r6_T = [pa(:,6); 1];
% r6 = T03 * r6_T;
% 
% th6omega_T = [sw(:,6); 0];
% th6omega_R = T05 * th6omega_T;

th6r = pa(:,6); %r6(1:3);
th6omega = sw(:,6); %th6omega_R(1:3);

th6U = th6p - th6r;
th6V = th6q(1:3) - th6r;

th6u_prime = th6U - th6omega*th6omega'*th6U;
th6v_prime = th6V - th6omega*th6omega'*th6V;


theta6 = round(atan2((th6omega'*Vector3Cross(th6u_prime,th6v_prime)'),...
    (th6u_prime'*th6v_prime)), 4);

theta6_d = theta6*180/pi;

screwPositions = [theta1_d; theta2_d; theta3_d; theta4_d; theta5_d; theta6_d]




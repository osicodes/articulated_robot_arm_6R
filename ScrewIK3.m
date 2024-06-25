%   Reference: 
%   [1] Imporved inverse kinematics Algorithm using Screw theory
%       for a Six-DOF robot manipulator
%       https://cyberleninka.org/article/n/690609/viewer
%   [2] Algorithmic approach to geometric solution of generalized 
%       Paden–Kahan subproblem and its extension
%       https://journals.sagepub.com/doi/pdf/10.1177/1729881418755157

tip=42.5;

d1=75.36; a1=35.36; a2=80.04; d4=100; d6 = 10+tip; 


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

qr = [pi/2; -pi/3; pi/6; -pi/6; -pi/3; 0];
% qr = [pi/2 pi/3 pi/2 pi/3 -pi/2 0]';

g0 = [0 0 1 a1+d4+d6;
    0 -1 0 0;
    1 0 0 d1+a2;
    0 0 0 1];

gst = FKinSpace(g0, s, qr); % Target pose transformation

%% INVERSE
% ----------------------------
q6 = [pa(:,6); 1];
pointOnAxis1 = pa(:,1);
q2 = pa(:,2);
q3 = pa(:,3);
q4 = pa(:,4);
% lo = [0; 0; 0];
d = [1; 0; 0];
omega1 = sw(:,1);

q_1 = gst * inv(g0) * q6;

% Remeber, q4, q5, q6 are same points.
% When theta1 is rotated, q6 will never change and rotate by theta1
% So to get theta1, move from q6 at home position to q6 at target position

home = q6(1:3);  % q6 at home position

P06=gst(1:3,4);
P05 = P06 - d6*gst(1:3,3); % q6 at target position

target = P05; 

th1p = home
th1q = target; % [target(1:2);home(3)]
th1r = pa(:,1)
th1omega = sw(:,1)

th1U = th1p - th1r
th1V = th1q - th1r

% th1u_prime and th1v_prime are projections on the x-y plane
% with theta1 between both vectors
th1u_prime = th1U - th1omega*th1omega'*th1U
th1v_prime = th1V - th1omega*th1omega'*th1V

theta1 = atan2(th1omega'*Vector3Cross(th1u_prime,th1v_prime)',...
    th1u_prime'*th1v_prime)

% --------------------------------
% theta2 and theta3
% 
% REF: Chen Q, Zhu S, Zhang X. Improved Inverse Kinematics Algorithm 
% Using Screw Theory for a Six-DOF Robot Manipulator. International 
% Journal of Advanced Robotic Systems. 2015;12(10). doi:10.5772/60834
% -------------------------------
% th2 is theta1, and th3 is theta2 in the reference
th23p = q6(1:3)
th23q = target; 
th2r = pa(:,2)
th3r = pa(:,3)
th23r23 = th3r - th2r % distance between axis 2 and 3
th2omega = sw(:,2)
th3omega = sw(:,3)

th23V = th23p - th3r
th23U = th23q - th2r

theta01 = atan2(th2omega'*Vector3Cross(th23U,th23r23)',...
    th23U'*th23r23)

phi1_num = norm(th23U)^2 + norm(th23r23)^2 - norm(th23V)^2
phi1_den = 2 * norm(th23U) * norm(th23r23)
phi1 = acos(phi1_num/phi1_den)

theta2 = phi1 + theta01 % OR theta2 = phi1 + theta01
% q = q_1(1:3,:);
% lo = [q(1); 0; q(3)];
% 
% s1 = [0; 0; 0];
% 
% s = s1 + omega1*(omega1'*(q - s1));
% % s = [0; 0; 0];
% 
% 
% Dl = (2*lo'*d - 2*d'*s)^2;
% Dr = 4*(norm(lo)^2 - 2*lo'*s - norm(q)^2 + 2*q'*s);
% D = Dl - Dr;
% 
% if D < 0
%     exit("no solution")
% elseif D == 0
%     t = lo'*d - d'*s;
%     qc = lo + t*d;
%     % Find theta1 using Paden–Kahan subproblem 1
%     u = qc - q1;
%     v = q - q1;
%     u_prime = u - sw(:,1)*sw(:,1)'*u;
%     v_prime = v - sw(:,1)*sw(:,1)'*v;
%     
%     theta1 = atan2(sw(:,1)'*Vector3Cross(u_prime,v_prime)',u_prime'*v_prime);
%     
%     % Find theta2 and theta3 using reference [1] pg. 4
% else  % D > 0
%     t1 = -1*(2*lo'*d - d'*s) + sqrt(D);
%     t2 = -1*(2*lo'*d - d'*s) - sqrt(D);
%     qc1 = lo + t1*d; %prefered
%     qc2 = lo + t2*d;
%     % Find theta1 using Paden–Kahan subproblem 1
%     %--------THETA 1------------------
%     u1 = qc1 - q1;
%     u2 = qc2 - q1;
%     v = q - q1;
%     
%     u_prime1 = u1 - sw(:,1)*sw(:,1)'*u1;
%     u_prime2 = u2 - sw(:,1)*sw(:,1)'*u2;
%     v_prime = v - sw(:,1)*sw(:,1)'*v;
%     
%     theta1 = atan2(sw(:,1)'*Vector3Cross(u_prime1,v_prime)',u_prime1'*v_prime) %prefered
%     % theta1 = atan2(sw(:,1)'*Vector3Cross(u_prime2,v_prime)',u_prime2'*v_prime)
%     
%     % Find theta2 and theta3 using reference [1] pg. 4
%     %--------THETA 2------------------
%     v1_1 = q4 - q3;
%     r23 = q2 - q3;
%     r32 = q3 - q2;
%     
%     % For u1_1
%     u1_1 = qc1 - q2;
%     th02_1 = atan2(sw(:,2)'*Vector3Cross(u1_1,r32)',u1_1'*r32);
%     
%     num2_1 = norm(u1_1)^2 + norm(r23)^2 - norm(v1_1)^2;
%     den2_1 = 2*norm(u1_1)*norm(r23);
%     
%     % theta2 = acos(num2_1/den2_1) + th02_1;
%     theta2 = acos(num2_1/den2_1) - th02_1
%     
%     % For u1_2
%     u1_2 = qc2 - q2;
%     th02_2 = atan2(sw(:,2)'*Vector3Cross(u1_2,r32)',u1_2'*r32);
%     
%     num2_2 = norm(u1_2)^2 + norm(r23)^2 - norm(v1_1)^2;
%     den2_2 = 2*norm(u1_2)*norm(r23);
%     
%     %theta2 = acos(num2_2/den2_2) + th02_2;
%     %theta2 = acos(num2_2/den2_2) - th02_2;
%     
%     %--------THETA 3------------------
%     th03_1 = atan2(sw(:,3)'*Vector3Cross(v1_1,r23)',v1_1'*r23);
%     
%     num3_1 = norm(v1_1)^2 + norm(r23)^2 - norm(u1_1)^2;
%     den3_1 = 2*norm(v1_1)*norm(r23);
%     
%     theta3 = th03_1 + acos(num3_1/den3_1)
% %     theta3 = th03_1 - acos(num3_1/den3_1)
% end


%--------THETA 4------------------



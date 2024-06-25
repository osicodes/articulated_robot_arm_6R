% 
% INVERSE KINEMATICS
% analytical with Euler angles
% 

function angles = myInverseKin6RAnalytical_EULER(x,y,z,phi,th,psi)

tip=42.5;
d1=75.3553; a1=35.3553; a2=80; d4=100; d6 = 10+tip;

%% https://homes.cs.washington.edu/~todorov/courses/cseP590/05_Kinematics.pdf  page 23
% w_t_T = world to task transformation
% w_b_T = world to robot base transformation
% b_e_T = base to end effector transformation
% e_t_T = end effector to task transformation
% 
% w_t_T = w_b_T * b_e_T * e_t_T
% 
% Aim is to obtain base to end effector transformation
% ------------------------
% 
% b_e_T = inv(w_b_T) * w_t_T * e_t_T

% World to base
w_b_T_pose = [0 0 0 0 0 0];
w_b_T = r_EULER_POSE(w_b_T_pose);

% End effector to task
e_t_T_pose = [0 0 0 0 0 0];
e_t_T = r_EULER_POSE(e_t_T_pose);

% world to task
% tg = [100.3553 50 70.8553 0 pi pi];
w_t_T_pose = [x y z phi th psi];
w_t_T = r_EULER_POSE(w_t_T_pose);

% base to end effector transformation
T06 = inv(w_b_T)*w_t_T*inv(e_t_T);

%%

% X = fprintf('T06 is');
% disp(X);
% disp(T06);

P06=T06(1:3,4);
%R06 = r_EULER(tg(4),tg(5),tg(6));

% % P05 = P06 - d6*R06*[0;0;1];
P05 = P06 - d6*T06(1:3,3);

% X = fprintf('P05 is');                                    m
%         
% disp(X); 
% disp(P05);

x = P05(1,1);
y = P05(2,1);
z = P05(3,1);

%--------theta 1------------
theta1 = atan2(y,x);%-atan2(0,sqrt(x^2+y^2));

%--------theta 2------------
r = sqrt(x^2 + y^2) - a1; 

z_P = z - d1; 

s = sqrt(r^2 + z_P^2);

alpha = atan2(z_P,r);

num1 = a2^2 + s^2 - d4^2;
den1 = 2 * a2 * s;
numden1 = round(num1/den1,4);

beta = acos(numden1);

AlphaPlusBeta = alpha + beta;
if (AlphaPlusBeta >= 0 && AlphaPlusBeta <= pi/2)
    theta2 = -1 * (pi/2 - alpha - beta);
end

if (AlphaPlusBeta > pi/2 && AlphaPlusBeta <= pi)
    theta2 = alpha + beta - pi/2;
end

%--------theta 3------------
num2 = a2^2 + d4^2 - s^2;
den2 = 2 * a2 * d4;
numden2 = round(num2/den2,4);
gamma = acos(numden2);

% theta3 = pi - gamma; % If the boundries of theta3 is 0 and 180
theta3 = gamma - pi/2; % If the boundries of theta3 is -90 and 90
%--------------------------------


A1 = dh(theta1,d1,a1,pi/2);
A2 = dh(theta2+pi/2,0,a2,0);
A3 = dh(theta3,0,0,pi/2);

T03 = A1*A2*A3;

R03 = [T03(1:3,1),T03(1:3,2),T03(1:3,3)];

trns_R03 = transpose(R03);

% Rotation matrix of joints 3 to 6 equals inverse/transpose of rotation 
% matrix of joints 1 to 3 * rotation matrix of 1 to 6
R36 = trns_R03*T06(1:3,1:3)

%%%------------------------------------------
% quatR03 = QUATERNION(R03)
% inv_R03s = QuatInverse(quatR03)
% % quat1 = QUATERNION(trns_R03)
% quat2 = QUATERNION(T06(1:3,1:3))
% 
% q1q2 = QuatMultiply(inv_R03s,quat2)
% 
% R36quat = r_QUATERNION(q1q2)
%%%------------------------------------------

% REF: https://opentextbooks.clemson.edu/wangrobotics/chapter/inverse-kinematics/
% REF: file:///C:/Users/osinachi/Downloads/351_27435_EE514_2016_1__2_1_0%208%20EE514%20Lec8%20inverse%20Kinematics.pdf


r23 = round(R36(2,3),4);
r13 = round(R36(1,3),4);

r32 = round(R36(3,2),4);
r31 = round(R36(3,1),4);

if (norm(R36(3,3)) == 1 || norm(R36(3,3)) == -1)
    %--------theta 5------------
    theta5 = round(asin(R36(3,3)),4);
    %--------theta 4------------
    theta4 = atan2(r23,r13);
    %--------theta 6------------
    theta6 = atan2(-r32,r31);
else
    %--------theta 5------------
    theta5 = atan2(-sqrt(1 - R36(3,3)^2), R36(3,3));
    %--------theta 4------------
    theta4 = atan2(-r23,-r13);
    %--------theta 6------------
    theta6 = atan2(-r32,r31);
end

%% 
% %--------theta 4------------
% r23 = round(R36(2,3),4);
% r13 = round(R36(1,3),4);
% theta4 = atan2(-r23,-r13); %original
% % theta4 = atan2(r23,r13);
% % theta4 = atan2(r23,-r13);
% 
% %--------theta 5------------
% %
% if (norm(R36(3,3)) == 1 || norm(R36(3,3)) == -1)
%     theta5 = round(asin(R36(3,3)),4);
% else
% %     theta5 = round(acos(R36(3,3)),4); %original
%     theta5 = atan2(-sqrt(1-R36(3,3)^2),R36(3,3));
% end
% % theta5 = round(acos(R36(3,3)),4); %original
% % theta5 = round(-acos(R36(3,3)),4); 
% % theta5 = atan2(sqrt(r13^2+r23^2),R36(3,3));
% 
% %--------theta 6------------
% r32 = round(R36(3,2),4);
% r31 = round(R36(3,1),4);
% 
% theta6 = atan2(-r32,r31); %original
% % theta6 = atan2(r32,-r31);
% %--------------------------------
%%

angles_rad = [theta1,theta2,theta3,theta4,theta5,theta6];
angles_deg = [theta1,theta2,theta3,theta4,theta5,theta6] * 180/pi;

angles = angles_rad; % angles_deg


end
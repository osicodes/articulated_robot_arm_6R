% Rough test 1

tip=42.5;
d1=75.3553; a1=35.3553; a2=80; d4=100; d6 = 10+tip;
%
Px=187.8553;Py=20;Pz=155.3553;a=pi/2;o=pi/2;n=0;
R06 = r_RPY(a,o,n);
P06=[Px;Py;Pz];
% % P05 = P06 - d6*R06*[0;0;1];
P05 = P06 - d6*R06(1:3,3);


%--------------------
x = P05(1,1);
y = P05(2,1);
z = P05(3,1);

theta1 = atan2(y,x);

ex = sqrt(x^2+y^2) - a1;
ez = z - d1;

beta = acos((a2^2+ez^2+ex^2-d4^2)/(2*a2*sqrt(ez^2+ex^2)));
alpha = atan2(ez,ex);

theta2 = pi/2 - beta - alpha;
theta3 = pi/2 - acos((a2^2+d4^2-ex^2-ez^2)/(2*a2*d4));
% theta2 = atan2(ez,ex)-atan2((d4*sin(theta3)),(a2+d4*cos(theta3)));
%----------------

%----!!!!!!!!!!!!!---------
% A1 = dh(theta1,d1,a1,pi/2);
% A2 = dh(theta2+pi/4,0,a2,0);
% A3 = dh(theta3+pi/2,0,0,pi/2);

A1 = dh(theta1,d1,a1,pi/2);
A2 = dh(theta2+pi/2,0,a2,0);
A3 = dh(theta3,0,0,pi/2);

T03 = A1*A2*A3;

R03 = [T03(1:3,1),T03(1:3,2),T03(1:3,3)];


%inv_R03 = inv(R03);
%new_inv_R36 = inv_R03*R06;

%
trns_R03 = transpose(R03);
%
new__trans_R36 = trns_R03*R06;
%----!!!!!!!!!!!!!---------

% % % theta4 = atan2(new__trans_R36(2,3),new__trans_R36(1,3));
theta4 = atan2(new__trans_R36(1,3),new__trans_R36(2,3));
%
% % theta5 = atan2( sqrt(new__trans_R36(1,3)^2 + new__trans_R36(2,3)^2),new__trans_R36(3,3));
%-----if theta5 is not zero
theta5 = atan2( new__trans_R36(3,3),sqrt(1- new__trans_R36(3,3)^2 ));
%theta5 = -acos(new__trans_R36(3,3));
% % theta6 = atan2(-new__trans_R36(3,2),new__trans_R36(3,1));
theta6 = atan2(-new__trans_R36(3,1),new__trans_R36(3,2));

myTheta = [theta1,theta2,theta3,theta4,theta5,theta6]

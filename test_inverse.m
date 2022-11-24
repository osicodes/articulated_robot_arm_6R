

tip=42.5;
d1=75.3553; a1=35.3553; a2=80; d4=100; d6 = 10+tip;
%
Px=50.8553;Py=50;Pz=50.3553;a=0;o=pi/4;n=0;
% R06 = r_RPY(a,o,n);
% 
R06 = r_EULER(a,o,n);
P06=[Px,Py,Pz];

T_F = [R06, P06';0 0 0 1];

%--------New Frames
% work frame
Wk_P = [0 0 0 0 0 0];
Wk_R = r_EULER(Wk_P(4),Wk_P(5),Wk_P(6));
Wk_T = [Wk_P(1),Wk_P(2),Wk_P(3)];
Wk_F = [Wk_R, Wk_T';0 0 0 1 ];
inv_Wk_F = inv(Wk_F);
% tool frame
Tl_P = [0 0 0 0 0 0];
Tl_R = r_EULER(Tl_P(4),Tl_P(5),Tl_P(6));
Tl_T = [Tl_P(1),Tl_P(2),Tl_P(3)];
Tl_F = [Tl_R, Tl_T';0 0 0 1 ];
inv_Tl_F = inv(Tl_F);

%--------New Frames End--------------
new_T_F = T_F * inv_Tl_F * inv_Wk_F;

% P05 = P06' - d6*R06*[0;0;1];
P05 = T_F(1:3,4) - d6*T_F(1:3,3);

%--------------------theta1,theta2,theta3---------------
x = P05(1,1);
y = P05(2,1);
z = P05(3,1);


ex = sqrt(x^2+y^2) - a1;
ez = z - d1;

beta = acos((a2^2+ez^2+ex^2-d4^2)/(2*a2*sqrt(ez^2+ex^2)));
alpha = atan2(ez,ex);


%
theta1 = atan2(y,x);
%
theta2 = pi/2 - beta - alpha; %From youtube
%
theta3 = pi/2 - acos((a2^2+d4^2-ex^2-ez^2)/(2*a2*d4));

%theta1 = atan2(y,x) - atan2(sqrt(ex^2-a1^2),a1);
%theta1 = atan2(y,x) - atan2(a1,sqrt(ex^2-a1^2));

%theta2 = atan2(ez,ex)-atan2((d4*sin(theta3)),(a2+d4*cos(theta3)));
%D = (ex^2+ez^2-a2^2-d4^2)/(2*a2*d4);
%theta3 = atan2(sqrt(1-D^2),D);


%--------------------theta1,theta2,theta3----end-----------

%----!!!!!!!!!!!!!---------
% A1 = dh(theta1,d1,a1,pi/2);
% A2 = dh(theta2+pi/4,0,a2,0);
% A3 = dh(theta3+pi/2,0,0,pi/2);

A1 = dh(theta1,d1,a1,pi/2);
A2 = dh(theta2+pi/2,0,a2,0);
A3 = dh(theta3,0,0,pi/2);

T03 = A1*A2*A3;

R03 = [T03(1:3,1),T03(1:3,2),T03(1:3,3)];


%
inv_R03 = inv(R03);
%
new_inv_R36 = inv_R03*R06;

%
trns_R03 = transpose(R03);
%
new__trans_R36 = trns_R03*R06;
%----!!!!!!!!!!!!!---------

theta4 = atan2(new__trans_R36(2,3),new__trans_R36(1,3));
%
theta5 = atan2( sqrt(new__trans_R36(1,3)^2 + new__trans_R36(2,3)^2),new__trans_R36(3,3));
%theta5 = -acos(new__trans_R36(3,3));
theta6 = atan2(-new__trans_R36(3,2),new__trans_R36(3,1));

myTheta = [theta1,theta2,theta3,theta4,theta5,theta6]

LinkOne = myBar(50,30,10);
LinkTwo = mySideBar(20,30,10);

tip=42.5;
%th1=theta1; th2=theta2; th3=theta3; th4=theta4; th5=theta5; th6=theta6;
d1=75.3553; a1=35.3553; a2=80; d4=100; d6 = 10+tip;
%syms th1 th2 th3 th4 th5 th6 a o n
%135 = 3/4*pi
th1=0.1061;th2=0.8033;th3=-0.5191;th4=1.5708;th5=0.2843;th6=-1.4647;
%q = [th1,th2,th3,th4,th5,th6]; 
% q = [1.4421   -0.0267    0.5210         0    2.6473    1.4421]; 

q = [0.9244   -0.4405    2.1554         0    1.4266    0.9245]; 
qr = q * -1;

% A1 = dh(q(1),d1,a1,pi/2);
% A2 = dh(q(2)+pi/4,0,a2,0);
% A3 = dh(q(3)+pi/2,0,0,pi/2);
% A4 = dh(q(4),d4,0,-pi/2);
% A5 = dh(q(5),0,0,pi/2);
% A6 = dh(q(6),d6,0,0);

% A1 = dh(q(1),d1,a1,-pi/2);
% A2 = dh(q(2)-pi/2,0,a2,0);
% A3 = dh(q(3),0,0,-pi/2);
% A4 = dh(q(4),d4,0,pi/2);
% A5 = dh(q(5),0,0,-pi/2);
% A6 = dh(q(6),d6,0,0);

A1 = dh(q(1),d1,a1,-pi/2);
A2 = dh(q(2)-pi/2,0,a2,0);
A3 = dh(q(3)-pi/2,0,0,-pi/2);
A4 = dh(q(4),d4,0,pi/2);
A5 = dh(q(5),0,0,-pi/2);
A6 = dh(q(6),d6,0,0);

% T06 = simplify(A1*A2*A3*A4*A5*A6)
T06 = A1*A2*A3*A4*A5*A6
[phi,th,psi] = inv_EULER(T06)

% T03 = A1*A2*A3;
% T36 = A4*A5*A6;
% 
% 
% R06 = simplify([T06(1:3,1),T06(1:3,2),T06(1:3,3)])
% R03 = simplify([T03(1:3,1),T03(1:3,2),T03(1:3,3)])
% RHS = simplify(transpose(R03)*R06)
% R36 = simplify([T36(1:3,1),T36(1:3,2),T36(1:3,3)])
% R06n = r_RPY(a,o,n)
% R36n = simplify(transpose(R03) * R06n)

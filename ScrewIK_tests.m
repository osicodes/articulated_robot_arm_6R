tip=42.5;

d1=75.36;
a1=35.36; 
a2=80.04; 
d4=100; 
d6 = 10+tip;
% syms a1 a2 d1 d4 th5 th6

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

lent = size(sv);
for i = 1:lent(2)
    sv(:,i) = Vector3Cross(-1*sw(:,i),pa(:,i));
end

s = [sw;sv];

% qr = [pi/4; -pi/6; -pi/6];
qr = [0 0 0 0 pi/6 0]';

g0 = [0 0 1 a1+d4+d6;
    0 -1 0 0;
    1 0 0 d1+a2;
    0 0 0 1];

g04 = [0 0 1 a1+d4;
    0 -1 0 0;
    1 0 0 d1+a2;
    0 0 0 1];

g05 = [0 -1  0 a1+d4;
       0  0 -1 0;
       1  0  0 d1+a2;
       0  0  0 1];

g03 = [0 0 1 a1;
    0 -1 0 0;
    1 0 0 d1+a2;
    0 0 0 1];



gst1 = FKinSpace(g0, s, qr) % Target pose transformation


q1 = [0 0 0 1]';
q2 = [a1 0 d1 1]';
q3 = [a1 0 d1+a2 1]';
q4 = [a1+d4 0 d1+a2 1]';

qout = [a1+d4+d6 0 d1+a2 1]';
qend = gst1(1:4,4);
             

exp1 = MatrixExp6(VecTose3(s(:, 1) * qr(1)));
exp2 = MatrixExp6(VecTose3(s(:, 2) * qr(2)));
exp3 = MatrixExp6(VecTose3(s(:, 3) * qr(3)));
exp4 = MatrixExp6(VecTose3(s(:, 4) * qr(4)));
exp5 = MatrixExp6(VecTose3(s(:, 5) * qr(5)));
exp6 = MatrixExp6(VecTose3(s(:, 6) * qr(6)));

left = exp6 * qend

right = inv(exp5) * inv(exp4) * inv(exp3) * ...
    inv(exp2) * inv(exp1) *gst1 * inv(g0) * qend


% left = exp4*exp5*exp6 * qout
% 
% right =   InvTransM(exp3) * InvTransM(exp2) * InvTransM(exp1) *...
%     gst1 * InvTransM(g0) * qout

t2 = exp1*q2;
t3 = exp1*exp2 * q3;
t4 = exp1*exp2*exp3*exp4*exp5*exp6 * q4;
tout = exp1*exp2*exp3*exp4*exp5*exp6 * qout;

point1 = [0; 0; 0];
point1_int = [0; 0; d1];
point2 = t2(1:3);
point3 = t3(1:3);
point4 = t4(1:3);
point_end = tout(1:3);


x = [point1(1) point1_int(1) point2(1) point3(1) point4(1) point_end(1)];
y = [point1(2) point1_int(2) point2(2) point3(2) point4(2) point_end(2)];
z = [point1(3) point1_int(3) point2(3) point3(3) point4(3) point_end(3)];

pl = plot3(x,y,z);
pl.Marker = '*';

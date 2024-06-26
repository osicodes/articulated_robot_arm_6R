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
qr = [pi/3 -pi/6 -pi/6 0 0 0]';

g0 = [0 0 1 a1+d4+d6;
    0 -1 0 0;
    1 0 0 d1+a2;
    0 0 0 1];



gst1 = FKinSpace(g0, s, qr) % Target pose transformation


q1 = [0 0 0 1]';
q2 = [a1 0 d1 1]';
q3 = [a1 0 d1+a2 1]';
q4 = [a1+d4 0 d1+a2 1]';

qout = [a1+d4+d6 0 d1+a2 1]';
             

exp1 = MatrixExp6(VecTose3(s(:, 1) * qr(1)));
exp2 = MatrixExp6(VecTose3(s(:, 2) * qr(2)));
exp3 = MatrixExp6(VecTose3(s(:, 3) * qr(3)));
exp4 = MatrixExp6(VecTose3(s(:, 4) * qr(4)));
exp5 = MatrixExp6(VecTose3(s(:, 5) * qr(5)));
exp6 = MatrixExp6(VecTose3(s(:, 6) * qr(6)));

left = exp1*exp2*exp3*exp4*exp5*exp6 * qout

right = gst * InvTransM(g0) * qout

g03 = [0 -1  0 a1;
       0  0 -1 0;
       1  0  0 d1+a2;
       0  0  0 1];

t03 = exp1*exp2*g03



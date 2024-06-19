%   Reference: 
%   [1] Imporved inverse kinematics Algorithm using Screw theory
%       for a Six-DOF robot manipulator
%       https://cyberleninka.org/article/n/690609/viewer
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
qr = [pi/2 pi/3 pi/2 pi/3 -pi/2 0]';

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

%------------INVERSE----------------
q6 = [pa(:,6); 1];
q1 = pa(:,1);
q2 = pa(:,2);
q3 = pa(:,3);
q4 = pa(:,4);
% lo = [0; 0; 0];
d = [1; 0; 0];
omega1 = sw(:,1);




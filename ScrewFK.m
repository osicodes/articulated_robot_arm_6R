tip=42.5;

d1=75.36; a1=35.36; a2=80.04; d4=100; d6 = 10+tip;
%pa = point on arbitrary screw axis
%sw = unit rotation of the screw axis
pa = zeros(3,6);
pa(:,1) = [0 0 0];
pa(:,2) = [a1 0 d1];
pa(:,3) = [a1 0 d1+a2];
pa(:,4) = [0 0 d1+a2];
pa(:,5) = [a1+d4 0 d1+a2];
pa(:,6) = [0 0 d1+a2];

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

M = [0 0 1 a1+d4+d6;
    0 -1 0 0;
    1 0 0 d1+a2;
    0 0 0 1];

qr = [-pi/6; 0; -pi/2; 0; 0; 0];

z = FKinSpace(M, s, qr)


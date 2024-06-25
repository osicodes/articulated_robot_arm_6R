tip=42.5;

d1=75.36;
a1=35.36; 
a2=80.04; 
d4=100; 
d6 = 10+tip;
% syms a1 a2 d1 d4 th5 th6

% pa = zeros(3,6);

pa = zeros(3,1);
% pa(:,1) = [0 0 0];
% pa(:,2) = [a1 0 d1];
pa(:,1) = [a1 0 d1+a2];
% pa(:,4) = [a1+d4 0 d1+a2];
% pa(:,5) = [a1+d4 0 d1+a2];
% pa(:,6) = [a1+d4 0 d1+a2];

% sw = zeros(3,6);
sw = zeros(3,1);
% sw(:,1) = [0 0 1];
% sw(:,2) = [0 -1 0];
sw(:,1) = [0 -1 0];
% sw(:,4) = [1 0 0];
% sw(:,5) = [0 -1 0];
% sw(:,6) = [1 0 0];

sv = zeros(3,1);

lent = size(sv);
for i = 1:lent(2)
    sv(:,i) = Vector3Cross(-1*sw(:,i),pa(:,i));
end

s = [sw;sv];

% qr = [0; 0; 0; 0; 0; 0];
% qr = [0 0 0 pi/3 -pi/2 0]';
qr = -pi/3;

% g0 = [0 0 1 a1+d4;
%     0 -1 0 0;
%     1 0 0 d1+a2;
%     0 0 0 1];

g0= [a1+d4 0 d1+a2 1]'


gst = FKinSpace(g0, s, qr) % Target pose transformation


% P06=gst(1:3,4);
% P05 = P06 - d6*gst(1:3,3); % q6 at target position
% 
% P03 = P05 - d4*gst(1:3,3);
% 
% % v1 = [a1+d4 - a1, d1+a2 - d1+a2, 0];
% % v2 = [85.3600 - a1, 68.7975 - d1+a2, 0];
% 
% v1 = pa(:,6) - pa(:,3);
% v2 = [85.3600; 0; 68.7975] - pa(:,3);
% 
% Theta = atan2(norm(cross(v1, v2)), dot(v1, v2))*180/pi;




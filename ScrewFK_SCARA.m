
% d0 is the distance from the ground to the end effector when the end
% effector shaft is fully down
d0=63; a1=225; a2=175;
%pa = point on arbitrary screw axis
%sw = a unit vector in the direction of the screw axis

% prismatic joint d3 is between 0 and 85mm
% 0 -> when the end effector shaft is fully down
% 85 -> when the end effector shaft is fully up

% theta1, theta2, d3, theta4

pa = zeros(3,3);
sw = zeros(3,4);

%% Axis 1 - revolute
pa(:,1) = [0 0 0];
sw(:,1) = [0 0 1];

%% Axis 2 - revolute
pa(:,2) = [a1 0 0];
sw(:,2) = [0 0 1];

%% Axis 3 - revolute
pa(:,3) = [a1+a2 0 0];
sw(:,3) = [0 0 -1];

%% Axis 4 - prismatic
v4 = [0 0 1]; % v4 is a unit vector pointing in the direction of translation
sw(:,4) = [0 0 0];

%% sv is linear velocity
sv = zeros(3,4);
% Linear velocities for revolute joints 1 to 3
for i = 1:3
    sv(:,i) = Vector3Cross(-1*sw(:,i),pa(:,i));
end

% Linear velocities for prismatic joints 4
sv(:,4) = v4;

s = [sw;sv];

M = [1  0  0 a1+a2;
     0 -1  0     0;
     0  0 -1    d0;
     0  0  0     1];



qr = [0; 0; 0; 20];


z = FKinSpace(M, s, qr)






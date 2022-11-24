function [cs_lft] = mySideBar(l_lft,w_lft,d_lft)

A_lft = linspace(-pi/2, pi/2, 25)';
B_lft = linspace(pi/2, 3*pi/2, 25)';

csRight_lft = [l_lft/2 + w_lft/2*cos(A_lft) w_lft/2*sin(A_lft)];
csLeft_lft = [-l_lft/2 + w_lft/2*cos(B_lft) w_lft/2*sin(B_lft)];


C_lft = linspace(3*pi/2, -pi/2, 10)';

%csRightHole_lft = [+l_lft/2 + d_lft/2*cos(C_lft) d_lft/2*sin(C_lft)];
csLeftHole_lft = [-l_lft/2 + d_lft/2*cos(C_lft) d_lft/2*sin(C_lft)];


%cs_lft = [csRight_lft; csLeft_lft; +l_lft/2 -w_lft/2; csRightHole_lft];


cs_lft = [csRight_lft; csLeft_lft; csLeftHole_lft; -l_lft/2 -w_lft/2];


%plot(cs_lft (:,1), cs_lft (:,2), 'Color', [0.6 0.6 0.6], 'Marker', '.',...
%'MarkerSize', 9, 'MarkerEdgeColor', [1 0 0]);

end
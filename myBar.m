function [cs] = myBar(l,w,d)

A = linspace(-pi/2, pi/2, 25)';
B = linspace(pi/2, 3*pi/2, 25)';

csRight = [l/2 + w/2*cos(A) w/2*sin(A)];
csLeft = [-l/2 + w/2*cos(B) w/2*sin(B)];

C = linspace(3*pi/2, -pi/2, 10)';

csLeftHole = [-l/2 + d/2*cos(C) d/2*sin(C)];
csRightHole = [+l/2 + d/2*cos(C) d/2*sin(C)];
csConnLine = [-l/2 -w/2; +l/2 -w/2];

cs = [csRight; csLeft; csLeftHole; ...
    csConnLine; csRightHole];

%plot(cs(:,1), cs(:,2), 'Color', [0.6 0.6 0.6], 'Marker', '.',...
%'MarkerSize', 9, 'MarkerEdgeColor', [1 0 0]);
end
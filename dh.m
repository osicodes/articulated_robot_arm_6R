function T = dh(th,d,a,alpha)

if alpha == pi/2
    x = 0;
elseif alpha == -pi/2
    x = 0;
else
    x = cos(alpha);
end

T = [cos(th) -sin(th)*x sin(th)*sin(alpha) a*cos(th);
    sin(th) cos(th)*x -cos(th)*sin(alpha) a*sin(th);
    0 sin(alpha) x d;
    0 0 0 1];
end
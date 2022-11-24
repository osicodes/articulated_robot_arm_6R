function R = r_RPY(a,o,n)
R = [cos(a)*cos(o) cos(a)*sin(o)*sin(n)-sin(a)*cos(n) cos(a)*sin(o)*cos(n)+sin(a)*sin(n);
     sin(a)*cos(o) sin(a)*sin(o)*sin(n)+cos(a)*cos(n) sin(a)*sin(o)*cos(n)-cos(a)*sin(n);
     -sin(o)       cos(o)*sin(n)                      cos(o)*cos(n)                     ];
end
function [a,o,n] = inv_RPY(M)
a = atan2(M(2,1),M(1,1));
o = atan2(-M(3,1),(M(1,1)*cos(a)+M(2,1)*sin(a)));
n = atan2((-M(2,3)*cos(a) + M(1,3)*sin(a)),(M(2,2)*cos(a) - M(1,2)*sin(a)));

end
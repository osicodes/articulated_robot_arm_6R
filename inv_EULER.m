function [phi,th,psi] = inv_EULER(M)
phi = atan2(M(2,3),M(1,3));
psi = atan2((-M(1,1)*sin(phi) + M(2,1)*cos(phi)),(-M(1,2)*sin(phi) + M(2,2)*cos(phi)));
th = atan2((M(1,3)*cos(phi) + M(2,3)*sin(phi)),M(3,3));
end
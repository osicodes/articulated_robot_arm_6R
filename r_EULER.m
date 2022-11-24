function R = r_EULER(phi,th,psi)
R = [cos(phi)*cos(th)*cos(psi)-sin(phi)*sin(psi)   -cos(phi)*cos(th)*sin(psi)-sin(phi)*cos(psi)   cos(phi)*sin(th);
     sin(phi)*cos(th)*cos(psi)+cos(phi)*sin(psi)   -sin(phi)*cos(th)*sin(psi)+cos(phi)*cos(psi)   sin(phi)*sin(th);
     -sin(th)*cos(psi)                             sin(th)*sin(psi)                               cos(th)];
end
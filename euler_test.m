% Convert rotation matrix to euler angles

R = [1         0    0;
         0   -1.0000         0;
    0 0   -1];

[phi,th,psi] = EULER(R)
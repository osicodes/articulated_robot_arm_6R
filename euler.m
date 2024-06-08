% Convert rotation matrix to euler angles

M = [1         0    0;
         0   -1.0000         0;
    0 0   -1];

[phi,th,psi] = inv_EULER(M)
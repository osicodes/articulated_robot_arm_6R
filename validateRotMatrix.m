function R = validateRotMatrix(M)
n = M(1,1)^2 + M(2,1)^2 + M(3,1)^2;
o = M(1,2)^2 + M(2,2)^2 + M(3,2)^2;
a = M(1,3)^2 + M(2,3)^2 + M(3,3)^2;
if (n>0.9999 && n<=1.0001  && o>0.9999 && o<=1.0001  && a>0.9999 && a<=1.0001)
    R = M;
else
    msg = 'Matrix is not a rotation matrix. For rotation matrices, determinant(M) is 1';
    error(msg)
end
end
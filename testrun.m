syms th1 th2 th3 th4 th5 th6 a o n 
syms d1 a1 a2 d4 
syms s1 s2 s3 s4 s5 s6 c1 c2 c3 c4 c5 c6 s23 c23
syms nx ny nx ox oy oz ax ay az px py pz

d6 = 0;

A1 = dh(th1,d1,a1,pi/2);
A2 = dh(th2+pi/2,0,a2,0);
A3 = dh(th3,0,0,pi/2);
A4 = dh(th4,d4,0,-pi/2);
A5 = dh(th5,0,0,pi/2);
A6 = dh(th6,d6,0,0);

% T06 = simplify(A1*A2*A3*A4*A5*A6);

px = a1*c1 - a2*c1*s2 - c1*d4*s2*s3 + c1*c2*c3*d4;
py = a1*s1 - a2*s1*s2 - d4*s1*s2*s3 + c2*c3*d4*s1;
pz = d1 + a2*c2 + c2*d4*s3 + c3*d4*s2;

T = ...
    [s6*(c4*s1 + c1*c2*s3*s4 + c1*c3*s2*s4) - c6*(c23*c1*s5 - c5*s1*s4 + c1*c2*c4*c5*s3 + c1*c3*c4*c5*s2),   c6*(c4*s1 + c1*c2*s3*s4 + c1*c3*s2*s4) + s6*(c23*c1*s5 - c5*s1*s4 + c1*c2*c4*c5*s3 + c1*c3*c4*c5*s2),   c23*c1*c5 - s5*(c1*c2*c4*s3 - s1*s4 + c1*c3*c4*s2),   a1*c1 - a2*c1*s2 + d4*c1*c2*c3 - d4*c1*s2*s3 + d6*s1*s4*s5 + d6*c1*c2*c3*c5 - d6*c1*c5*s2*s3 - d6*c1*c2*c4*s3*s5 - d6*c1*c3*c4*s2*s5;
     s6*(c2*s1*s3*s4 - c1*c4 + c3*s1*s2*s4) - c6*(c23*s1*s5 + c1*c5*s4 + c2*c4*c5*s1*s3 + c3*c4*c5*s1*s2),   c6*(c2*s1*s3*s4 - c1*c4 + c3*s1*s2*s4) + s6*(c23*s1*s5 + c1*c5*s4 + c2*c4*c5*s1*s3 + c3*c4*c5*s1*s2),   c23*c5*s1 - s5*(c1*s4 + c2*c4*s1*s3 + c3*c4*s1*s2),   a1*s1 - a2*s1*s2 + d4*c2*c3*s1 - d6*c1*s4*s5 - d4*s1*s2*s3 + d6*c2*c3*c5*s1 - d6*c5*s1*s2*s3 - d6*c2*c4*s1*s3*s5 - d6*c3*c4*s1*s2*s5;
     - c6*(s23*s5 - c23*c4*c5) - c23*s4*s6,                                            s6*(s23*s5 - c23*c4*c5) - c23*c6*s4,                                              s23*c5 + c23*c4*s5,                        d1 + a2*c2 + d4*c2*s3 + d4*c3*s2 + d6*c2*c5*s3 + d6*c3*c5*s2 + d6*c2*c3*c4*s5 - d6*c4*s2*s3*s5;
     0,                                                                                                                 0,                                                                                                                 0,                                                               1];


% [s6*(c4*s1 + c1*c2*s3*s4 + c1*c3*s2*s4) - c6*(c23*c1*s5 - c5*s1*s4 + c1*c2*c4*c5*s3 + c1*c3*c4*c5*s2),   c6*(c4*s1 + c1*c2*s3*s4 + c1*c3*s2*s4) + s6*(c23*c1*s5 - c5*s1*s4 + c1*c2*c4*c5*s3 + c1*c3*c4*c5*s2),   c23*c1*c5 - s5*(c1*c2*c4*s3 - s1*s4 + c1*c3*c4*s2),   a1*c1 - a2*c1*s2 + d4*c1*c2*c3 - d4*c1*s2*s3 + d6*s1*s4*s5 + d6*c1*c2*c3*c5 - d6*c1*c5*s2*s3 - d6*c1*c2*c4*s3*s5 - d6*c1*c3*c4*s2*s5]
% [s6*(c2*s1*s3*s4 - c1*c4 + c3*s1*s2*s4) - c6*(c23*s1*s5 + c1*c5*s4 + c2*c4*c5*s1*s3 + c3*c4*c5*s1*s2),   c6*(c2*s1*s3*s4 - c1*c4 + c3*s1*s2*s4) + s6*(c23*s1*s5 + c1*c5*s4 + c2*c4*c5*s1*s3 + c3*c4*c5*s1*s2),   c23*c5*s1 - s5*(c1*s4 + c2*c4*s1*s3 + c3*c4*s1*s2),   a1*s1 - a2*s1*s2 + d4*c2*c3*s1 - d6*c1*s4*s5 - d4*s1*s2*s3 + d6*c2*c3*c5*s1 - d6*c5*s1*s2*s3 - d6*c2*c4*s1*s3*s5 - d6*c3*c4*s1*s2*s5]
% [- c6*(s23*s5 - c23*c4*c5) - c23*s4*s6,                                            s6*(s23*s5 - c23*c4*c5) - c23*c6*s4,                                              s23*c5 + c23*c4*s5,                        d1 + a2*c2 + d4*c2*s3 + d4*c3*s2 + d6*c2*c5*s3 + d6*c3*c5*s2 + d6*c2*c3*c4*s5 - d6*c4*s2*s3*s5]
% [0,                                                                                                                 0,                                                                                                                 0,                                                               1]

% T_new =
%  
% [s6*(c4*s1 + c1*c2*s3*s4 + c1*c3*s2*s4) - c6*(c1*c23*s5 - c5*s1*s4 + c1*c2*c4*c5*s3 + c1*c3*c4*c5*s2), s6*(c1*c23*s5 - c5*s1*s4 + c1*c2*c4*c5*s3 + c1*c3*c4*c5*s2) + c6*(c4*s1 + c1*c2*s3*s4 + c1*c3*s2*s4), c1*c5*c23 - s5*(c1*c2*c4*s3 - s1*s4 + c1*c3*c4*s2), a1*c1 - a2*c1*s2 - c1*d4*s2*s3 + c1*c2*c3*d4]
% [s6*(c2*s1*s3*s4 - c1*c4 + c3*s1*s2*s4) - c6*(c1*c5*s4 + c23*s1*s5 + c2*c4*c5*s1*s3 + c3*c4*c5*s1*s2), s6*(c1*c5*s4 + c23*s1*s5 + c2*c4*c5*s1*s3 + c3*c4*c5*s1*s2) + c6*(c2*s1*s3*s4 - c1*c4 + c3*s1*s2*s4), c5*c23*s1 - s5*(c1*s4 + c2*c4*s1*s3 + c3*c4*s1*s2), a1*s1 - a2*s1*s2 - d4*s1*s2*s3 + c2*c3*d4*s1]
% [                                                               - c6*(s5*s23 - c4*c5*c23) - c23*s4*s6,                                                                  s6*(s5*s23 - c4*c5*c23) - c6*c23*s4,                                 c5*s23 + c4*c23*s5,             d1 + a2*c2 + c2*d4*s3 + c3*d4*s2]
% [                                                                                                   0,                                                                                                    0,                                                  0,                                            1]
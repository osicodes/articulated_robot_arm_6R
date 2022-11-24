function R = r_EULER_T(pos)
R = [cos(pos(4))*cos(pos(5))*cos(pos(6))-sin(pos(4))*sin(pos(6))   -cos(pos(4))*cos(pos(5))*sin(pos(6))-sin(pos(4))*cos(pos(6))   cos(pos(4))*sin(pos(5))         pos(1);
     sin(pos(4))*cos(pos(5))*cos(pos(6))+cos(pos(4))*sin(pos(6))   -sin(pos(4))*cos(pos(5))*sin(pos(6))+cos(pos(4))*cos(pos(6))   sin(pos(4))*sin(pos(5))         pos(2);
     -sin(pos(5))*cos(pos(6))                             sin(pos(5))*sin(pos(6))                               cos(pos(5))                  pos(3);
     0                                             0                                              0                        1];
end
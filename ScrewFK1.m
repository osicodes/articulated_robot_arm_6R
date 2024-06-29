function T = ScrewFK1(g0, s, qr)
% g0 is the initial configuration matrix
% s is the screw axis of each joint
% qr is the initial configuration joint angles

exp1 = MatrixExp6(VecTose3(s(:, 1) * qr(1)));
exp2 = MatrixExp6(VecTose3(s(:, 2) * qr(2)));
exp3 = MatrixExp6(VecTose3(s(:, 3) * qr(3)));
exp4 = MatrixExp6(VecTose3(s(:, 4) * qr(4)));
exp5 = MatrixExp6(VecTose3(s(:, 5) * qr(5)));
exp6 = MatrixExp6(VecTose3(s(:, 6) * qr(6)));

T = exp1*exp2*exp3*exp4*exp5*exp6 * g0;

end
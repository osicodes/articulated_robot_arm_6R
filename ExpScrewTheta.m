% The exponential of screw times theta
tip=42.5;

d1=75.36; a1=35.36; a2=80.04; d4=100; d6 = 10+tip;

pa = zeros(3,6);
pa(:,1) = [0 0 0];
pa(:,2) = [a1 0 d1];
pa(:,3) = [a1 0 d1+a2];
pa(:,4) = [a1+d4 0 d1+a2];
pa(:,5) = [a1+d4 0 d1+a2];
pa(:,6) = [a1+d4 0 d1+a2];

sw = zeros(3,6);
sw(:,1) = [0 0 1];
sw(:,2) = [0 -1 0];
sw(:,3) = [0 -1 0];
sw(:,4) = [1 0 0];
sw(:,5) = [0 -1 0];
sw(:,6) = [1 0 0];

sv = zeros(3,6);
for i = 1:6
    sv(:,i) = Vector3Cross(-1*sw(:,i),pa(:,i));
end

Slist = [sw;sv];

thetalist = [0 0 -pi/3 0 pi/6 0]';


exp1 = MatrixExp6(VecTose3(Slist(:, 1) * thetalist(1)));
exp2 = MatrixExp6(VecTose3(Slist(:, 2) * thetalist(2)));
exp3 = MatrixExp6(VecTose3(Slist(:, 3) * thetalist(3)));
exp4 = MatrixExp6(VecTose3(Slist(:, 4) * thetalist(4)));

q7b = exp3*[pa(:,4); 1]

gt = [0 0 1 a1*cos(thetalist(1));
    0 -1 0 a1*sin(thetalist(1));
    1 0 0 0;
    0 0 0 1];

gst_0 = [0 0 1 a1+d4+d6;
    0 -1 0 0;
    1 0 0 d1+a2;
    0 0 0 1];

gst = FKinSpace(gst_0, Slist, thetalist);


p3b = inv(exp1) * gt * gst * inv(gst_0) * [pa(:,4); 1]


th2p = q7b(1:3); %q3;
th2q = p3b(1:3);
th2r = pa(:,2);
th2omega = sw(:,2);

th2U = th2p - th2r;
th2V = th2q - th2r;

% th1u_prime and th1v_prime are projections on the x-y plane
% with theta1 between both vectors
th2u_prime = th2U - th2omega*th2omega'*th2U;
th2v_prime = th2V - th2omega*th2omega'*th2V;

theta2 = round(atan2(th2omega'*Vector3Cross(th2u_prime,th2v_prime)',...
    th2u_prime'*th2v_prime)*180/pi, 4)


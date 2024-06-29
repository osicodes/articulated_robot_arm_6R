function T = InvTransM(M)
rot = M(1:3,1:3);
trans = M(1:3,4);

transX = -1 * (trans(1) * rot(1,1) + trans(2) * rot(2,1) + trans(3) * rot(3,1));
transY = -1 * (trans(1) * rot(1,2) + trans(2) * rot(2,2) + trans(3) * rot(3,3));
transZ = -1 * (trans(1) * rot(1,3) + trans(2) * rot(2,3) + trans(3) * rot(3,3));

T = [rot' [transX;transY;transZ];
     0 0 0 1];
end
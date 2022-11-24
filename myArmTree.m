Ts = 0.001;
[ArmTree,ArmInfo] = importrobot('myArm.slx');

showdetails(ArmTree);

qInitial = homeConfiguration(ArmTree);
ik = inverseKinematics('RigidBodyTree', ArmTree);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'Body7';
point = [100,100,100];
qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
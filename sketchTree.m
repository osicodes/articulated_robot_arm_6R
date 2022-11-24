% 
% tip=42.5;
% d1=75.3553; a1=35.3553; a2=80; d4=100; d6 = 10+tip;

tip=0.425;
d1=0.753553; a1=0.353553; a2=0.80; d4=1.00; d6 = 0.10+tip;


 dhparams = [a1, -pi/2, d1,    0;
             a2,     0, 0, -pi/2;
             0,  -pi/2, 0, -pi/2;
             0,   pi/2, d4,   0;
             0,  -pi/2, 0,   0;
             0,      0, d6,   0];


%  dhparams = [a1, -pi/2, d1,    0;
%              a2,     0, 0, -pi/2;
%              0,  -pi/2, 0,    0;
%              0,   pi/2, d4,   0;
%              0,  -pi/2, 0,   0;
%              0,      0, d6,   0];

       %       a  alpha      d   theta
% dhparams = [0   	pi/2	0   	0;
%             0.4318	0       0       0
%             0.0203	-pi/2	0.15005	0;
%             0   	pi/2	0.4318	0;
%             0       -pi/2	0   	0;
%             0       0       0       0];
        
sketchrobot = rigidBodyTree;

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(sketchrobot,body1,'base')
addBody(sketchrobot,body2,'body1')
addBody(sketchrobot,body3,'body2')
addBody(sketchrobot,body4,'body3')
addBody(sketchrobot,body5,'body4')
addBody(sketchrobot,body6,'body5')

% showdetails(sketchrobot)
% 
% show(sketchrobot);
% % axis([-200,200,-200,200,-200,200])
% % axis([-5,5,-5,5,-5,5])
% axis([-2,2,-2,2,-2,2])
% 
% % axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])


aik = analyticalInverseKinematics(sketchrobot);
showdetails(aik)

generateIKFunction(aik,'robotIK');
eePosition = [0.103553 0.80 1.908553];
eePose = trvec2tform(eePosition);
% hold on
% plotTransforms(eePosition,tform2quat(eePose))
% hold off
show(sketchrobot)
ikConfig = robotIK(eePose); % Uses the generated file

% show(sketchrobot,ikConfig(1,:));
% hold on
% plotTransforms(eePosition,tform2quat(eePose))
% hold off



% gik = generalizedInverseKinematics;
% gik.RigidBodyTree = sketchrobot;
% gik.ConstraintInputs = {'position','aiming'};
% 
% posTgt = constraintPositionTarget('body6');
% posTgt.TargetPosition = [0.203553 0.80 1.908553];
% 
% aimCon = constraintAiming('body6');
% aimCon.TargetPoint = [0.0 0.0 0.0];
% 
% fixOrientation = constraintOrientationTarget('body6');
% fixOrientation.TargetOrientation = [0 1 0 0];
% 
% q0 = homeConfiguration(sketchrobot); % Initial guess for solver
% [qrr,solutionInfo] = gik(q0,posTgt,aimCon);
% show(sketchrobot,qrr);
% q = [qrr.JointPosition]
% title(['Solver status: ' solutionInfo.Status])
% % axis([-0.75 0.75 -0.75 0.75 -0.5 1])
% 
% axis([-3,3,-3,3,-3,3])







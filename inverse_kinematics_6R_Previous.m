function inverse_kinematics_6R(block)


setup(block);

%endfunction


%%   Set up the basic characteristics of the S-function block such as:
function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        = [1 6];
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% Override output port properties
block.OutputPort(1).Dimensions       = [1 6];
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
block.SampleTimes = [0 0];

% Specify the block simStateCompliance. 
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all block methods. 
%% -----------------------------------------------------------------

block.RegBlockMethod('Outputs', @Outputs);     % Required
% block.RegBlockMethod('SetInputPortSamplingMode', @SetInputPortFrameData);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%% Outputs:
function Outputs(block)

tip=42.5;
d1=75.3553; a1=35.3553; a2=80; d4=100; d6 = 10+tip;

% Work frame
wf = [0 0 0 0 0 0];
wf_T = r_EULER_T(wf);
%Tool frame
tf = [0 0 0 0 0 0];
tf_T = r_EULER_T(tf);

%Position frame
% tg = [100.3553 50 70.8553 0 pi pi];
tg = [block.InputPort(1).Data(1,1) block.InputPort(1).Data(1,2) block.InputPort(1).Data(1,3) block.InputPort(1).Data(1,4) block.InputPort(1).Data(1,5) block.InputPort(1).Data(1,6)];
tg_T = r_EULER_T(tg);

T06 = inv(wf_T)*tg_T*inv(tf_T);

P06=T06(1:3,4);

P05 = P06 - d6*T06(1:3,3);

x = P05(1,1);
y = P05(2,1);
z = P05(3,1);


%--------theta 1------------
theta1 = atan2(y,x);%-atan2(0,sqrt(x^2+y^2));

%--------theta 2------------
r = sqrt(x^2 + y^2) - a1; % OR r = sqrt(x^2 + y^2) + a1

z_P = z - d1; 

s = sqrt(r^2 + z_P^2);

alpha = atan(z_P/r);

num1 = a2^2 + s^2 - d4^2;
den1 = 2 * a2 * s;
nd1 = round(num1/den1,4);
beta1 = acos(nd1);
% beta2 = -acos(nd1);

theta2 = pi/2 - alpha - beta1; % OR theta2 = pi/2 - alpha - beta2

%--------theta 3------------
num2 = a2^2 + d4^2 - s^2;
den2 = 2 * a2 * d4;
nd2 = round(num2/den2,4);
gamma1 = acos(nd2);
gamma2 = -acos(nd2);

theta3 = pi - gamma1; % OR theta3 = pi/2 - gamma2
%--------------------------------


A1 = dh(theta1,d1,a1,-pi/2);
A2 = dh(theta2-pi/2,0,a2,0);
A3 = dh(theta3-pi/2,0,0,-pi/2);

T03 = A1*A2*A3;

R03 = [T03(1:3,1),T03(1:3,2),T03(1:3,3)];

trns_R03 = transpose(R03);
R36 = trns_R03*T06(1:3,1:3);


%--------theta 4------------
r23 = round(R36(2,3),4);
r13 = round(R36(1,3),4);
theta4 = atan2(-r23,-r13);

%--------theta 5------------
%
theta5 = round(acos(R36(3,3)),4); % OR theta5 = -acos(R36(3,3))
%theta5 = atan2(sqrt(r13^2+r23^2),R36(3,3));

%--------theta 6------------
r32 = round(R36(3,2),4);
r31 = round(R36(3,1),4);
%theta6 = atan2(-R36(3,2),R36(3,1));
theta6 = atan2(-r32,r31);
%--------------------------------

trd = [theta1,theta2,theta3,theta4,theta5,theta6];

block.OutputPort(1).Data = trd;



%end Outputs

function R = r_EULER_T(pos)
R = [cos(pos(4))*cos(pos(5))*cos(pos(6))-sin(pos(4))*sin(pos(6))   -cos(pos(4))*cos(pos(5))*sin(pos(6))-sin(pos(4))*cos(pos(6))   cos(pos(4))*sin(pos(5))         pos(1);
     sin(pos(4))*cos(pos(5))*cos(pos(6))+cos(pos(4))*sin(pos(6))   -sin(pos(4))*cos(pos(5))*sin(pos(6))+cos(pos(4))*cos(pos(6))   sin(pos(4))*sin(pos(5))         pos(2);
     -sin(pos(5))*cos(pos(6))                             sin(pos(5))*sin(pos(6))                               cos(pos(5))                  pos(3);
     0                                             0                                              0                        1];

function T = dh(th,d,a,alpha)

if alpha == pi/2
    x = 0;
elseif alpha == -pi/2
    x = 0;
else
    x = cos(alpha);
end

T = [cos(th) -sin(th)*x sin(th)*sin(alpha) a*cos(th);
    sin(th) cos(th)*x -cos(th)*sin(alpha) a*sin(th);
    0 sin(alpha) x d;
    0 0 0 1];


%% Set sampling of the input ports
% function SetInputPortFrameData(block, idx, fd)
% 
% block.InputPort(idx).SamplingMode = fd;
% for i = 1:block.NumOutputPorts
%     block.OutputPort(i).SamplingMode = fd;
% end

%end 

%%
%% Terminate:
%%
function Terminate(block)

%end Terminate


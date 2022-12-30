% % (x,y) -> Center
% % r -> Radius
% [x,y]=circle(160,0,20); 
% z=repelem(8,201);
% 
% cir = [x',y',z'];
% 
% trajList = [0;0;0;0;0;0];
% for i = 1:numel(cir(:,1))
%     trajList(1,i) = cir(i,1);
%     trajList(2,i) = cir(i,2);
%     trajList(3,i) = cir(i,3);
%     trajList(4,i) = 0; 
%     trajList(5,i) = 3.1416;
%     trajList(6,i) = 3.1416;
% end
% 
% function [xunit,yunit] = circle(x,y,r) 
% hold on 
% th = 0:pi/100:2*pi; 
% xunit = r * cos(th) + x; 
% yunit = r * sin(th) + y; 
% plot(xunit, yunit); 
% hold off
% end
 
  
%-----------------Cartesian traj------------------------
Xstart = [ 0.0000   -0.0000    1.0000 187.8553
   -0.0000   -1.0000         0      0
    1.0000   -0.0000   -0.0000 155.3553
         0         0         0 1.0000];

Xend = [0.4505   -0.2169   -0.8660   80.3521;
       -0.4339   -0.9010    0.0000   50.9937;
       -0.7803    0.3758   -0.5000  80.8569;
         0         0         0    1.0000];
     
Tf = 5;
N = 200;
method = 3;
trajList = [0;0;0;0;0;0];
cir = [0;0;0];
jointList = zeros(6,N);  
traj = Cartesian_Trajectory(Xstart, Xend, Tf, N, method);

for i = 1: N

    [phi,th,psi] = inv_EULER(traj{1,i});
    cir(i,1) = traj{1,i}(1,4);
    cir(i,2) = traj{1,i}(2,4);
    cir(i,3) = traj{1,i}(3,4);
    trajList(1,i) = traj{1,i}(1,4);
    trajList(2,i) = traj{1,i}(2,4);
    trajList(3,i) = traj{1,i}(3,4);
    trajList(4,i) = phi;
    trajList(5,i) = th;
    trajList(6,i) = psi;
    q = InverseKin6R(traj{1,i}(1,4),traj{1,i}(2,4),traj{1,i}(3,4),phi,th,psi);
    for j = 1:6
        jointList(j,i) = q(1,j);
    end
    
%     A1 = dh(q(1),d1,a1,pi/2); A2 = dh(q(2)+pi/2,0,a2,0); A3 =
%     dh(q(3),0,0,pi/2); A4 = dh(q(4),d4,0,-pi/2); A5 = dh(q(5),0,0,pi/2);
%     A6 = dh(q(6),d6,0,0);
%     
%     T06 = A1*A2*A3*A4*A5*A6; disp(T06);
%     
%     cir(i,1) = T06(1,4); cir(i,2) = T06(2,4); cir(i,3) = T06(3,4);
end
jointList; %angles for each joint
trajList;
cir;




% Rstart = [ 1.0000    0.0000    0.0000 
%            0.0000   -1.0000         0 
%            0.0000    0.0000   -1.0000 ];
%        
% Rend = [ 0.0000    0.0000    1.0000 
%            0.0000   1.0000         0 
%            1.0000    0.0000   0.0000];
% 
% Xcir = 160;
% Ycir = 0;
% Tf = 5;
% N = 100;
% r = 20;
% 
% trajList = [0;0;0;0;0;0];
% cir = [0;0;0];
% jointList = zeros(6,N); 
% traj = CircularTrajectory(Xcir, Ycir, Tf, N, r, Rstart, Rend);
% 
% for i = 1: N
%     [phi,th,psi] = inv_EULER(traj{1,i});
%     cir(i,1) = traj{1,i}(1,4);
%     cir(i,2) = traj{1,i}(2,4);
%     cir(i,3) = traj{1,i}(3,4);
%     trajList(1,i) = traj{1,i}(1,4);
%     trajList(2,i) = traj{1,i}(2,4);
%     trajList(3,i) = traj{1,i}(3,4);
%     trajList(4,i) = phi;
%     trajList(5,i) = th;
%     trajList(6,i) = psi;
%     q = InverseKin6R(traj{1,i}(1,4),traj{1,i}(2,4),traj{1,i}(3,4),phi,th,psi);
%     for j = 1:6
%         jointList(j,i) = q(1,j);
%     end
% end
% see; %angles for each joint
% jointList;
% cir


%-------------------joint traj----------------------
% X1 = [187.8553 0 155.3553 0 1.5708 -3.1416];
%      
% X2 = [80.3521 0 80.8569 -1.5708 2.4758 0.3082];
% X3 = [162.6345 0 8.7132 0 3.1416 3.1416];
% 
% [j1, c1] = jointFunc(X1,X2);
% [j2, c2] = jointFunc(X2,X3);
% 
% jointList = [j1 j2(:,2:end)]; 
% cir = [c1; c2(2:end,:)];
% 
% function [jointLis, ci]= jointFunc(Xstart, Xend)
% tip=42.5;
% d1=75.3553; a1=35.3553; a2=80; d4=100; d6 = 10+tip;
% 
% thetastart = InverseKin6R(Xstart(1),Xstart(2),Xstart(3),Xstart(4),Xstart(5),Xstart(6))';
% thetaend = InverseKin6R(Xend(1),Xend(2),Xend(3),Xend(4),Xend(5),Xend(6))';
% 
% % thetastart = [0; 0; 0; 0; 0; 0];
% % thetaend = [0; -pi/4; 0; 0; -pi/4; 0];
% Tf = 4;
% N = 100;
% method = 1;
% ci = [0;0;0];
% 
% vmax = 5;%0.25e-4;
% amax = 0.5;%0.75e-10;
% i = Xstart(1:3);
% f = Xend(1:3);
% eucPathLength = norm(f - i);
% traj = JointTrajectory(thetastart, thetaend, Tf, N, method, vmax, amax, eucPathLength);
% % traj = Trapezoidal_Trajectory(thetastart, thetaend, vmax, amax, eucPathLength, N);
% jointLis = traj';
% q=traj;
% for i = 1: N
%     A1 = dh(q(i,1),d1,a1,pi/2);
%     A2 = dh(q(i,2)+pi/2,0,a2,0);
%     A3 = dh(q(i,3),0,0,pi/2);
%     A4 = dh(q(i,4),d4,0,-pi/2);
%     A5 = dh(q(i,5),0,0,pi/2);
%     A6 = dh(q(i,6),d6,0,0);
%     
%     T06 = A1*A2*A3*A4*A5*A6;
% %     disp(T06);
%     
%     ci(i,1) = T06(1,4);
%     ci(i,2) = T06(2,4);
%     ci(i,3) = T06(3,4);
% end
% end
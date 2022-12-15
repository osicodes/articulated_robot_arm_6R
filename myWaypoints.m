% (x,y) -> Center
% r -> Radius
[x,y]=circle(90,50,60); 
z=repelem(150,201);

cir = [x',y',z'];

trajList = [0;0;0;0;0;0];
for i = 1:numel(cir(:,1))
    trajList(1,i) = cir(i,1);
    trajList(2,i) = cir(i,2);
    trajList(3,i) = cir(i,3);
    trajList(4,i) = 0; 
    trajList(5,i) = 0.2546;
    trajList(6,i) = 1.5708;
end

function [xunit,yunit] = circle(x,y,r) 
hold on 
th = 0:pi/100:2*pi; 
xunit = r * cos(th) + x; 
yunit = r * sin(th) + y; 
plot(xunit, yunit); 
hold off
end
 
% trajList = [0;0;0;0;0;0];
% see = zeros(N,6);    
% 
% Xstart = [ 0.0000   -0.0000    1.0000 187.8553
%    -0.0000   -1.0000         0      0
%     1.0000   -0.0000   -0.0000 155.3553
%          0         0         0 1.0000];
% Xend = [1.0000    3.1416   -0.0000   80.3521;
%    -0.0000   -1.0000   -0.7854   50.9937;
%    -1.5708    0.7854   -1.0000  80.8569;
%          0         0         0    1.0000];
% Tf = 5;
% N = 8;
% method = 3;
% 
% traj = CartesianTrajectory(Xstart, Xend, Tf, N, method);
% 
% for i = 1: N
% %     ddf(1,i) = traj{1,i}(1,4);
%     [phi,th,psi] = inv_EULER(traj{1,i});
%     trajList(1,i) = traj{1,i}(1,4);
%     trajList(2,i) = traj{1,i}(2,4);
%     trajList(3,i) = traj{1,i}(3,4);
%     trajList(4,i) = phi;
%     trajList(5,i) = th;
%     trajList(6,i) = psi;
%     trd = InverseKin6R(traj{1,i}(1,4),traj{1,i}(2,4),traj{1,i}(3,4),phi,th,psi);
%     for j = 1:6
%         see(i,j) = trd(1,j);
%     end
% end
% see %angles for each joint
% trajList

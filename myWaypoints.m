%(x,y) -> Center
%r -> Radius
% [x,y]=circle(100,100,30); z=repelem(100,21);
% 
% function [xunit,yunit] = circle(x,y,r) hold on th = 0:pi/10:2*pi; xunit =
% r * cos(th) + x; yunit = r * sin(th) + y; plot(xunit, yunit); hold off
% end

Xstart = [1.0000    0.7854   -0.7854  0.3555;
   -0.7854   -1.0000   -0.7854   49.9963;
   -0.7854    3.1416   -1.0000   170.8581;
         0         0         0    1.0000];
Xend = [1.0000    3.1416   -0.0000   80.3521;
   -0.0000   -1.0000   -0.7854   50.9937;
   -1.5708    0.7854   -1.0000  80.8569;
         0         0         0    1.0000];
Tf = 6;
N = 80;
method = 5;

traj = CartesianTrajectory(Xstart, Xend, Tf, N, method);

trajList = [0;0;0;0;0;0];
see = 0;
for i = 1: N
%     ddf(1,i) = traj{1,i}(1,4);
    [phi,th,psi] = inv_EULER(traj{1,i});
    trajList(1,i) = traj{1,i}(1,4);
    trajList(2,i) = traj{1,i}(2,4);
    trajList(3,i) = traj{1,i}(3,4);
    trajList(4,i) = phi;
    trajList(5,i) = th;
    trajList(6,i) = psi;
    trd = InverseKin6R(traj{1,i}(1,4),traj{1,i}(2,4),traj{1,i}(3,4),phi,th,psi);
    for j = 1:6
        see(i,j) = trd(1,j);
    end
end
see %angles for each joint
trajList


% 
% Start= 
% 
%  1.0000    0.0000   -0.0000  100.3555
%    -0.0000   -1.0000   -0.0000   49.9963
%    -0.0000    0.0000   -1.0000   70.8581
%          0         0         0    1.0000
% 
% end=
% 
% 1.0000    0.0000   -0.0000   10.3521
%    -0.0000   -1.0000   -0.0000   79.9937
%    -0.0000    0.0000   -1.0000  190.8569
%          0         0         0    1.0000
% 

t = [0 0 0 0 0 0];
for i = 1:20
    t(1,i) = see(i,1);
end
% t = [0:0.1:2*pi];
a = sin(t);
% a = sin(2*pi*60*t)
plot(t,a)
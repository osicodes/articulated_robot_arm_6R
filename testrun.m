Tf=5;
N=8;
tg=Tf/(N-1);
s=zeros(1,N);
% points = (1,1) and (4,5)
norm = 5;

for i=1:N
    t=tg*(i-1);
    s = (1-cos(pi*t / Tf))/2;
%     s_prime = 6 * t / (Tf) ^ 2 - 6 * (t) ^ 2 / (Tf) ^ 3; %Cubic
%     s = 3 * (t / Tf) ^ 2 - 2 * (t / Tf) ^ 3; %Cubic
    s(i)=s;
end
s
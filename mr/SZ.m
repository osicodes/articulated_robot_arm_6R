function [S] = SZ(T)
a = 4;
b = 4;

for r = 1:a
    for c = 1:b
        S(r,c) = 1/(r+c-1);
    end
end



end
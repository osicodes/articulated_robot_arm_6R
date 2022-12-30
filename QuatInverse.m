function inv = QuatInverse(qs)
inv = [qs(1) -qs(2) -qs(3) -qs(4)]/(qs(1)^2+qs(2)^2+qs(3)^2+qs(4)^2);
   
end
function s = Vector3Cross(a,b)
s(1) = a(2)*b(3) - a(3)*b(2);
s(2) = a(3)*b(1) - a(1)*b(3);
s(3) = a(1)*b(2) - a(2)*b(1);
end
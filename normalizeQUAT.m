function qn = normalizeQUAT(q)
den = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
qn = q/den;
end
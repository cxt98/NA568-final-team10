function H = jacobian(theta)
% theta = omega*dt;
H = 0;
for k = 0:20
    H = H+(-1)^k/factorial(k+1)*R3_to_so3(theta)^k;
end
end

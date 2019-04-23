function output = SO3_InverseRightJacobian(theta)
theta_norm = norm(theta);
A = R3_to_so3(theta);
if theta_norm == 0
    output = eye(3);
    return;
end
output = eye(3) + 0.5*A + (1/theta_norm + (1+cos(theta_norm))/(2*theta_norm*sin(theta_norm)))*A^2;
end

function output = SO3_LeftJacobian(theta)
theta_norm = norm(theta);
A = R3_to_so3(theta);
if theta_norm == 0
    output = eye(3);
    return;
end
output = eye(3) + ((1-cos(theta_norm))/theta_norm^2)*A + ((theta_norm-sin(theta_norm))/theta_norm^3)*A^2;
end
function [r_ij, J_ij] = preintegration_factor_rotm(R_i,R_j,dR_ij)

r_ij = so3_to_R3(logm((dR_ij)'*R_i'*R_j));
J_ij = [-SO3_InverseRightJacobian(r_ij)*R_j'*R_i,SO3_InverseRightJacobian(r_ij)];

end

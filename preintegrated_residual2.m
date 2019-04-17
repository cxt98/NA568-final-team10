function [r_dRij, r_dvij, r_dpij] = preintegrated_residual2(x_prev, x_guess, dR_ij, dv_ij, dp_ij, dt_ij, g)

R_i = exp_map(x_prev(1:3));
v_i = x_prev(4:6);
p_i = x_prev(7:9);

R_j = exp_map(x_guess(1:3));
v_j = x_guess(4:6);
p_j = x_guess(7:9);

% Sum total time increment (IMU timekeeping)
sdt_ij = sum(dt_ij);

% Equation (45) - residual calculation
r_dRij = log_map((dR_ij)' * R_i' * R_j);

r_dvij = R_i' * (v_j - v_i - g*sdt_ij) - (dv_ij);

r_dpij = R_i' * (p_j - p_i - v_i*sdt_ij - 0.5*g*sdt_ij^2) - (dp_ij);
end
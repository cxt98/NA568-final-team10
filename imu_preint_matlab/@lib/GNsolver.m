function x_target = GNsolver(x_init,dt_ij,dR_ij,dv_ij,dp_ij,nkey,Sigma,g)
% initial guess
x_target = x_init;
% Gauss-Newton solver
max_iter = 1000;
iter = 0;
eps_Jr = 1e-6;
while iter < max_iter
    iter = iter + 1;
    for i = 1:nkey-1
        % compute residual using new state vector 
        % x_target changed every loop, hopefully less and less
        % dR_ij, dv_ij, dp_ij are the measurement, they do not change
        [r_dRij, r_dvij, r_dpij] = preintegrated_residual2(x_target(9*i-8:9*i),x_target(9*i+1:9*i+9), dR_ij, dv_ij, dp_ij, dt_ij, g);
        r(9*i-8:9*i,1) = Sigma{i}\[r_dRij;r_dvij;r_dpij];
    end
    % solve normal equations
    Jr = -A' * r;
    dx = (A' * A) \ Jr;
    x_target = x_target + dx;
    % check if converged
    norm(Jr)
    if norm(Jr) < eps_Jr
        break;
    end
end
end
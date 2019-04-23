function x_target = Gauss_Newton_rotm(x_est,dR_ij,nkey,Sigma)
max_iter = 1000;
iter = 0;
eps_Jr = 1e-3;

n_state = 3;

A = zeros(n_state*nkey,n_state*nkey);
r = zeros(n_state*nkey,1);
A(1:n_state,1:n_state) = chol(Sigma(:,:,1),'lower')\eye(n_state);
r(1:n_state,1) = zeros(n_state,1);
while iter < max_iter
    iter = iter + 1;
    for i = 1:nkey-1
        R_i = x_est.R(:,:,i);    % SO(3)
        R_j = x_est.R(:,:,i+1);  % SO(3)
        
        res_idx = n_state*i+(1:n_state);
        
        r_dR_ij = so3_to_R3(logm((dR_ij)'*R_i'*R_j));
        J_ij = [-SO3_InverseRightJacobian(r_dR_ij)*R_j'*R_i,...
            SO3_InverseRightJacobian(r_dR_ij)];

        L = chol(Sigma(:,:,i+1),'lower');
        
        A(res_idx,n_state*(i-1)+1:n_state*(i+1)) = L\J_ij;
        r(res_idx,1) = L\r_dR_ij;
    end
    A = sparse(A);
    
    % solve normal equations
    Jr = -A' * r;
    dx = (A' * A)\Jr;               % Lie algebra
    if norm(dx) > 10
        disp('error')
    end
    for i = 1:nkey
        dphi = dx((i-1)*n_state+(1:3));
        x_est.R(:,:,i) = x_est.R(:,:,i)*expm(R3_to_so3(dphi));
    end
    % check if converged
    if norm(Jr) < eps_Jr
        disp(iter)
        x_target = x_est;
        break;
    end
end
end
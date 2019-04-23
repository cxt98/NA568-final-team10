function x_target = Gauss_Newton(x_est,dt_ij,dR_ij,dv_ij,dp_ij,nkey,Sigma,g)
max_iter = 1000;
iter = 0;
eps_Jr = 0.1;

n_state = 9;

A = zeros(n_state*nkey,n_state*nkey);
r = zeros(n_state*nkey,1);
A(1:n_state,1:n_state) = chol(Sigma(:,:,1),'lower')\eye(n_state);
r(1:n_state,1) = zeros(n_state,1);

while iter < max_iter
    iter = iter + 1;
    
    for i = 1:nkey-1
        j = i+1;
        R_i = x_est.R(:,:,i);   % SO(3)
        v_i = x_est.v(:,i);
        p_i = x_est.p(:,i);
        R_j = x_est.R(:,:,j);   % SO(3)
        v_j = x_est.v(:,j);
        p_j = x_est.p(:,j);
        
        res_idx = n_state*i+(1:n_state);
        [r_ij, J_ij] = preintegration_factor(R_i, R_j, v_i, v_j, p_i, ...
            p_j, dR_ij, dv_ij, dp_ij, dt_ij, g);      
        L = chol(Sigma(:,:,i+1),'lower');
        A(res_idx,n_state*(i-1)+1:n_state*i) = L\J_ij(1:9,1:9);
        A(res_idx,n_state*i+1:n_state*i+n_state) = L\J_ij(1:9,10:18);
        r(res_idx,1) = L\r_ij;
    end
    A = sparse(A);
    
    % solve normal equations
    Jr = -A' * r;
    dx = (A' * A)\Jr;               % Lie algebra
    if norm(dx) > 10
        disp('error')
    end
    for k = 1:nkey
        dphi = dx((k-1)*n_state+(1:3));
        dv = dx((k-1)*n_state+(4:6));
        dp = dx((k-1)*n_state+(4:6));
        x_est.v(:,k) = x_est.v(:,k)+x_est.R(:,:,k)*dv;
        x_est.p(:,k) = x_est.p(:,k)+x_est.R(:,:,k)*dp;
        x_est.R(:,:,k) = x_est.R(:,:,k)*expm(R3_to_so3(dphi));
    end
    % check if converged
    if norm(Jr) < eps_Jr
        disp(iter)
        x_target = x_est;
        break;
    end
end
end
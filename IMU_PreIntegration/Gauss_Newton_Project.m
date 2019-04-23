function x_target = Gauss_Newton_Project(x_est,dtij,dRij,dvij,dpij,nkey,Sigma,g)
max_iter = 1000;
iter = 0;
eps_Jr = 0.001;

n_state = 9;

A = zeros(n_state*nkey,n_state*nkey);
r = zeros(n_state*nkey,1);
A(1:n_state,1:n_state) = chol(Sigma(:,:,1),'lower')\eye(n_state);
r(1:n_state,1) = zeros(n_state,1);

while iter < max_iter
    iter = iter + 1;
    
    for i = 1:nkey-1
        j = i+1;
        Ri = x_est.R(:,:,i);   % SO(3)
        vi = x_est.v(:,i);
        pi = x_est.p(:,i);
        Rj = x_est.R(:,:,j);   % SO(3)
        vj = x_est.v(:,j);
        pj = x_est.p(:,j);
        
        res_idx = n_state*i+(1:n_state);
        
        [r_ij, J_ij] = preintegration_factor(Ri, Rj, vi, vj, pi, ...
            pj, dRij, dvij, dpij, dtij, g); 
        
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
        dp = dx((k-1)*n_state+(7:9));
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
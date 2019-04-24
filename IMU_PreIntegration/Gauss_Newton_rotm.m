function x_target = Gauss_Newton_rotm(x_est,dRij,nkey,Sigma)
max_iter = 1000;
iter = 0;
eps_Jr = 0.001;

n_state = 3;
n_measurement = 3;

A = zeros(3+n_measurement*(nkey-1),nkey*n_state);
r = zeros(3+n_measurement*(nkey-1),1);

A(1:n_measurement,1:n_state) = chol(Sigma(:,:,1),'lower')\eye(n_state);
r(1:n_measurement,1) = zeros(n_measurement,1);

while iter < max_iter
    iter = iter+1;
    
    for i = 1:nkey-1
        j = i+1;
        res_idx = 3+(i-1)*3+(1:3);
        Ri = x_est.R(:,:,i);
        Rj = x_est.R(:,:,j);
        
        r_dRij = so3_to_R3(logm(dRij'*Ri'*Rj));
        
        L = chol(eye(3),'lower');
        
        r(res_idx,1) = L\r_dRij;
        A(res_idx,i*3-2:i*3) = L\(-SO3_InverseRightJacobian(r_dRij)*Rj'*Ri);
        A(res_idx,j*3-2:j*3) = L\SO3_InverseRightJacobian(r_dRij);
    end
    A = sparse(A);
    % solve normal equations
    Jr = -A'*r;
    dx = (A'*A)\Jr;         % Lie algebra
    for k = 1:nkey
        dphi = dx(k*3-2:k*3);
        x_est.R(:,:,k) = x_est.R(:,:,k)*expm(R3_to_so3(dphi));
    end
     % check if converged
    if norm(Jr) < eps_Jr
        disp('converge')
        disp(iter)
        x_target = x_est;
        break
    end
end
end


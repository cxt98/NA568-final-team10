function Sigma = noise_propagation_ij_rotm(t,omega,accel,Sigma)
dt = diff(t);               % 1 by 20
theta = zeros(3,length(dt));% 3 by 20
theta_init = zeros(3,1);    % 3 by 1
n = 3;
for k = 1:length(dt)
    theta(:,k) = theta_init+jacobian(theta_init)^-1*omega(:,k)*dt(k);
    
    Sigma_g = 0.0035*2*pi/360*sqrt(2000)*eye(3);
    Sigma_a = 0.14e-3*9.80665*sqrt(2000)*eye(3);
    
    Rk = expm(R3_to_so3(theta(:,k)));
    
    Ak = eye(n);
    Ak(1:3,1:3) = eye(3)-dt(k)/2*R3_to_so3(omega(:,k));
%      Ak(4:6,1:3) = 1/2*Rk*R3_to_so3(-accel(:,k))*jacobian(theta(:,k))*dt(k)^2;
%      Ak(4:6,7:9) = eye(3)*dt(k);
%      Ak(4:6,1:3) = Rk*R3_to_so3(-accel(:,k))*jacobian(theta(:,k))*dt(k);

    
    Bk = zeros(n,3);
%    Bk(4:6,:) = 1/2*Rk*dt(k)^2;
%    Bk(4:6,:) = Rk*dt(k);
    
    Ck = zeros(n,3);
    Ck(1:3,:) = jacobian(theta(:,k))^-1*dt(k);
    
    Sigma = Ak*Sigma*Ak'+Bk*Sigma_a*Bk'+Ck*Sigma_g*Ck';
    
    theta_init = theta(:,k);
end
end

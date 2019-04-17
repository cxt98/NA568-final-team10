function Sigma = noise_propagation_ij(t,omega,accel,Sigma)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments.
    %
    % Contributors: Yizhou Lu, Joseph Yates
    % Last Edited: 20190416

    dt = diff(t);                % 1 by 20
    theta = zeros(3,length(dt)); % 3 by 20
    theta_init = zeros(3,1);     % 3 by 1
    
    for k = 1:length(dt)
        theta(:,k) = theta_init+jacobian(theta_init)^-1*omega(:,k)*dt(k);

        Sigma_g = 0.0035*2*pi/360*sqrt(2000)*eye(3);
        Sigma_a = 0.14e-3*9.80665*sqrt(2000)*eye(3);

        Rk = expm(R3_to_so3(theta(:,k)));

        Ak = eye(9);
        Ak(1:3,1:3) = eye(3)-dt(k)/2*R3_to_so3(omega(:,k));
        Ak(4:6,1:3) = 1/2*Rk*vector_to_so3(-accel(:,k))* ...
            jacobian(theta(:,k))*dt(k)^2;
        Ak(4:6,7:9) = eye(3)*dt(k);
        Ak(7:9,1:3) = Rk*vector_to_so3(-accel(:,k))* ...
            jacobian(theta(:,k))*dt(k);

        Bk = zeros(9,3);
        Bk(4:6,:) = 1/2*Rk*dt(k)^2;
        Bk(7:9,:) = Rk*dt(k);

        Ck = zeros(9,3);
        Ck(1:3,:) = jacobian(theta(:,k))^-1*dt(k);

        Sigma = Ak*Sigma*Ak'+Bk*Sigma_a*Bk'+Ck*Sigma_g*Ck';

        theta_init = theta(:,k);
    end
end

% function H = jacobian(theta)
% % theta = omega*dt;
% H = 0;
% for k = 0:20
%     H = H+(-1)^k/factorial(k+1)*R3_to_so3(theta)^k;
% end
% end
% 
% function so3matrix = R3_to_so3(theta)
% % theta = omega*dt
% Gx = skew([1;0;0]);
% Gy = skew([1;0;0]);
% Gz = skew([1;0;0]);
% so3matrix = theta(1)*Gx+theta(2)*Gy+theta(3)*Gz;
% end
% 
% function X = skew(x)
% X = [0,    - x(3),   x(2);   
%      x(3),   0,     -x(1);
%     -x(2),   x(1),   0];
% end

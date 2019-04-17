clc;clear;close all

load('mocap_vectorNav_data.mat')
dt = 0.0005;
plot3(mocap.position(1,:),mocap.position(2,:),mocap.position(3,:))
axis equal;grid on;hold on
% Assume keyframe is at 1001,2001,3001,...,131001,132001
keyframe = 1:1000:132001;
nkey = length(keyframe);
Traj = zeros(3,nkey);
% initial condition (use quaternion for rotation matrix)
X.R = quat2rotm(imu.orientation(:,1)');
X.P = zeros(3,1);
X.V = zeros(3,1);
R{1} = X.R;
tic
for i = 1:nkey-1
    keyframe_i = keyframe(i);
    keyframe_j = keyframe(i+1);
    % predict the state vector at the next keyframe
    X_next = factor(X,imu,dt,keyframe_i,keyframe_j);
    Traj(:,i) = X_next.P(1:3);
    R{i+1} = X_next.R;
    % reassign initial condition
    X = X_next;
end
plot3(Traj(1,1:end-1),Traj(2,1:end-1),Traj(3,1:end-1))
toc
% R_int = quat2rotm(imu.orientation(:,1)');
% v_int = zeros(3,1);
% P_int = zeros(3,1);
% Traj_int = zeros(3,length(imu.orientation));
% g = [0;0;-9.80665];
% for i = 1:length(imu.orientation)
%     R_int = R_int*rotation_matrix(imu.angularVelocity(:,i),dt,'Exp');
%     v_int = v_int+g*dt+R_int*(imu.linearAcceleration(:,i)-[0;0;0])*dt;
%     P_int = P_int+v_int*dt+0.5*g*dt^2+0.5*R_int*imu.linearAcceleration(:,i)*dt^2;
%     Traj_int(:,i) = P_int;
% end
% plot3(Traj_int(1,:),Traj_int(2,:),Traj_int(3,:))
% toc

function Xj = factor(Xi,imu,dt,keyframe_i,keyframe_j)
Ri = Xi.R;
Pi = Xi.P;
Vi = Xi.V;

g = [0;0;-9.80665];

tij = imu.time(keyframe_j)-imu.time(keyframe_i);

theta = zeros(3,length(keyframe_i:1:keyframe_j));
p = zeros(3,length(keyframe_i:1:keyframe_j));
v = zeros(3,length(keyframe_i:1:keyframe_j));
k = 0;
R_temp = Ri;
for idx = keyframe_i:1:keyframe_j
    if k == 0
        % initialize the node with imu data at keyframe i
        theta(:,k+1) = imu.angularVelocity(:,idx)*dt;
        p(:,k+1) = (imu.linearAcceleration(:,idx)*dt^2)/2;
        v(:,k+1) = imu.linearAcceleration(:,idx)*dt;
        % Sigma = zeros(9,9);
    else
        Rk = expm(R3_to_so3(theta(:,k)));
        R_temp = R_temp*Rk;
        % move on to the next node
        theta(:,k+1) = theta(:,k)+jacobian(theta(:,k))^-1*imu.angularVelocity(:,idx)*dt;
        p(:,k+1) = p(:,k)+v(:,k)*dt+Rk*(imu.linearAcceleration(:,idx)*dt^2)/2;
        v(:,k+1) = v(:,k)+Rk*imu.linearAcceleration(:,idx)*dt;
        Sigma_next = noise_propagation(imu,dt,k,Rk,theta(:,k),Sigma);
        Sigma = Sigma_next
    end
    k = k+1;
end
Xj.R = Ri*expm(R3_to_so3(theta(:,end)));
Xj.P = Pi+Vi*tij+(g*tij^2)/2+Ri*p(:,end);
Xj.V = Vi+g*tij+Ri*v(:,end);
end

function Sigma_next = noise_propagation(imu,dt,k,Rk,theta,Sigma)
Sigma_g = diag(0.0035*sqrt(2000)*randn(3,1));
Sigma_a = diag(0.14e-3*9.80665*sqrt(2000)*randn(3,1));

Ak = eye(9);
Ak(1:3,1:3) = eye(3)-dt/2*R3_to_so3(imu.angularVelocity(:,k));
Ak(4:6,1:3) = 1/2*Rk*R3_to_so3(-imu.linearAcceleration(:,k))*jacobian(theta)*dt^2;
Ak(4:6,7:9) = eye(3)*dt;
Ak(7:9,1:3) = Rk*R3_to_so3(-imu.linearAcceleration(:,k))*jacobian(theta)*dt;

Bk = zeros(9,3);
Bk(4:6,1:3) = 1/2*Rk*dt^2;
Bk(7:9,1:3) = Rk*dt;

Ck = zeros(9,3);
Ck(1:3,1:3) = jacobian(theta)^-1*dt;
Sigma_next = Ak*Sigma*Ak'+Bk*Sigma_a*Bk'+Ck*Sigma_g*Ck';
end

function H = jacobian(theta)
% theta = omega*dt;
H = 0;
for k = 0:20
    H = H+(-1)^k/factorial(k+1)*R3_to_so3(theta)^k;
end
end

function se3matrix = R6_to_se3(xi)
% xi = [omega*dt;v*dt];
se3matrix = [R3_to_so3(xi(1:3)),xi(4:6);zeros(1,4)];
end

function so3matrix = R3_to_so3(theta)
% theta = omega*dt
Gx = skew([1;0;0]);
Gy = skew([0;1;0]);
Gz = skew([0;0;1]);
so3matrix = theta(1)*Gx+theta(2)*Gy+theta(3)*Gz;
end

function X = skew(x)
X = [0,    - x(3),   x(2);   
     x(3),   0,     -x(1);
    -x(2),   x(1),   0];
end


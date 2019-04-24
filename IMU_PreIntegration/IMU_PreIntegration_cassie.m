clc;clear;close all
load('mocap_vectorNav_data.mat')
% construct ground truth R,v,p using motion capture data
t_gt = mocap.time;                              % 1 by 6159
p_gt = mocap.position;                          % 3 by 6159
v_gt = [0,diff(p_gt(1,:))./diff(t_gt);
        0,diff(p_gt(2,:))./diff(t_gt);
        0,diff(p_gt(3,:))./diff(t_gt)];         % 3 by 6159
% find the nearest time point of mocap in imu, setting them as keyframes
keyframe = knnsearch(imu.time',t_gt');          % 6159 by 1
nkey = length(keyframe);                        % 6159
R_gt = quat2rotm(imu.orientation(:,keyframe)'); % 3 by 3 by 6159

% gravity
g = [0 0 -9.80665]';

% number of measurement in each keyframe
% only rotational matrix is optimized in this example, therefore, n = 3
n = 3;

% residual covariance, assume initial sigma to be 0.001*identity
Sigmaij = zeros(n,n,nkey);
Sigmaij(:,:,1) = 0.001*eye(n);

% state vector, initialize with ground truth of mocap
x_target.R(:,:,1) = R_gt(:,:,1);
x_target.v(:,1) = v_gt(:,1);
x_target.p(:,1) = p_gt(:,1);

% initial guess, initialize with ground truth of mocap
R_est(:,:,1) = R_gt(:,:,1);
v_est(:,1) = v_gt(:,1);
p_est(:,1) = p_gt(:,1);

range = 1:200;
for i = range
    j = i+1;

    [tij,omegaij,accelij] = keyframe_segmentation(imu,keyframe(i),keyframe(j));
    
    [dtij,dRij,dvij,dpij] = relative_motion(tij,omegaij,accelij);
    % initialize the next state with IMU measurement and current state 
    R_est(:,:,j) = R_est(:,:,i)*dRij;
    v_est(:,j) = R_est(:,:,i)*dvij+v_est(:,i)+g*sum(dtij);
    p_est(:,j) = R_est(:,:,i)*dpij+p_est(:,i)+v_est(:,i)*sum(dtij)+1/2*g*sum(dtij.^2);
    
    
    x_target.R(:,:,j) = x_target.R(:,:,i)*dRij;
    x_target.v(:,j) = x_target.R(:,:,i)*dvij+x_target.v(:,i)+g*sum(dtij);
    x_target.p(:,j) = x_target.R(:,:,i)*dpij+x_target.p(:,i)+x_target.v(:,i)*sum(dtij)+1/2*g*sum(dtij.^2);
    
%     Sigmaij(:,:,j) = noise_propagation_ij(tij,omegaij,accelij,Sigmaij(:,:,i));
%     x_target = Gauss_Newton(x_target,dtij,dRij,dvij,dpij,j,Sigmaij,g);

    Sigmaij(:,:,j) = noise_propagation_ij_rotm(tij,omegaij,accelij,Sigmaij(:,:,i));
    x_target = Gauss_Newton_rotm(x_target,dRij,j,Sigmaij); 
end
figure(1)
plot3(p_gt(1,range),p_gt(2,range),p_gt(3,range))
grid on;axis equal;hold on
plot3(p_est(1,:),p_est(2,:),p_est(3,:))
plot3(x_target.p(1,:),x_target.p(2,:),x_target.p(3,:))

figure(2)
plot(p_gt(1,range),p_gt(2,range))
grid on;axis equal;hold on
plot(p_est(1,:),p_est(2,:))
plot(x_target.p(1,:),x_target.p(2,:))
legend('Ground Truth','Initial Guess','Optimization')

%% Absolute Motion
% dt = diff(imu.time);
% R = R_gt(:,:,1);
% v = zeros(3,1);
% p = zeros(3,length(dt));
% for j = 1:length(dt)
%     R = R*expm(R3_to_so3((imu.angularVelocity(:,j))*dt(j)));
%     v = v+g*dt(j)+R*(imu.linearAcceleration(:,j))*dt(j);
%     p(:,j+1) = p(:,j)+v*dt(j)+1/2*g*dt(j)^2+1/2*R*(imu.linearAcceleration(:,j))*dt(j)^2;
% end
% plot(p(1,:),p(2,:))











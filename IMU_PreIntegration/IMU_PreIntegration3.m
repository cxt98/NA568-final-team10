clc;clear
load('mocap_vectorNav_data.mat')
% construct ground truth R,v,p
t_gt = mocap.time;                              % 1 by 6159
p_gt = mocap.position;                          % 3 by 6159
v_gt = [0,diff(p_gt(1,:))./diff(t_gt);
        0,diff(p_gt(2,:))./diff(t_gt);
        0,diff(p_gt(3,:))./diff(t_gt)];         % 3 by 6159
% find the nearest time point of mocap in imu
keyframe = knnsearch(imu.time',t_gt');          % 6159 by 1
nkey = length(keyframe);                        % 6159
R_gt = quat2rotm(imu.orientation(:,keyframe)'); % 3 by 3 by 6159

% gravity
g = [0 0 -9.80665]';

n = 9;
% residual covariance, guess initial sigma to be identity
Sigmaij = zeros(n,n,nkey);
Sigmaij(:,:,1) = 0.001*eye(n);

% state vector, initialize with ground truth of mocap
x_target.R(:,:,1) = R_gt(:,:,1);
x_target.v(:,1) = v_gt(:,1);
x_target.p(:,1) = p_gt(:,1);

% guess for R, initialize with ground truth of mocap
R_est(:,:,1) = R_gt(:,:,1);
v_est(:,1) = v_gt(:,1);
p_est(:,1) = p_gt(:,1);

for i = 1:nkey-1
    j = i+1;

    [tij,omegaij,accelij] = keyframe_segmentation(imu,keyframe(i),keyframe(j));

    [dtij,dRij,dvij,dpij] = relative_motion(tij,omegaij,accelij);
    R_est(:,:,j) = dRij*R_est(:,:,i);
    v_est(:,j) = R_est(:,:,i)*dvij+v_est(:,i)+g*sum(dtij);
    p_est(:,j) = R_est(:,:,i)*dpij+p_est(:,i)+v_est(:,i)*sum(dtij)+1/2*g*sum(dtij.^2);
    
%     Sigmaij(:,:,j) = noise_propagation_ij(tij,omegaij,accelij,Sigmaij(:,:,i));
%     
%     x_target.R(:,:,j) = R_est(:,:,j);
%     x_target.v(:,j) = v_est(:,j);
%     x_target.p(:,j) = p_est(:,j);
%     
%     x_target = Gauss_Newton(x_target,dtij,dRij,dvij,dpij,j,Sigmaij,g);
%     % x_target = Gauss_Newton_rotm2(x_target,dRij,j,Sigmaij);
    
end

plot(p_est(1,:),p_est(2,:))
grid on;axis equal;hold on
plot(p_gt(1,:),p_gt(2,:))

% quat_tar = rotm2quat(x_target.R)';
% quat_gt = rotm2quat(R_gt)';
% quat_est = rotm2quat(R_est)';
% figure(1)
% plot(1:101,quat_tar(1,:),'r')
% hold on
% plot(1:101,quat_tar(2,:),'b')
% plot(1:101,quat_tar(3,:),'k')
% plot(1:101,quat_tar(4,:),'g')
% plot(1:101,quat_gt(1,1:101),'r--')
% plot(1:101,quat_gt(2,1:101),'b--')
% plot(1:101,quat_gt(3,1:101),'k--')
% plot(1:101,quat_gt(4,1:101),'g--')
% plot(1:101,quat_est(1,1:101),'r:')
% plot(1:101,quat_est(2,1:101),'b:')
% plot(1:101,quat_est(3,1:101),'k:')
% plot(1:101,quat_est(4,1:101),'g:')
% legend('target_w','target_x','target_y','target_z','gt_w','gt_x','gt_y','gt_z')






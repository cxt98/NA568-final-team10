clc;clear
load('g_w.mat')
load('IMU.mat')
load('result.mat')

imu.time = IMU(:,1)';
imu.omega = IMU(:,2:4)';
imu.accel = IMU(:,5:7)';

twb = result(:,2:4)';
quatwb(:,1) = result(:,8);
quatwb(:,2:4) = result(:,5:7);
Rwb = quat2rotm(quatwb);

keyframe = 1:10:500;
nkey = length(keyframe);            % 50

t_gt = imu.time(keyframe);          % 1 by 50                                     % 1 by 6159
p_gt = twb;                         % 3 by 51

% gravity
g = g_w';

% residual covariance, guess initial sigma to be identity
Sigmaij = zeros(9,9,nkey);
Sigmaij(:,:,1) = 0.001*eye(9);

% state vector, initialize with ground truth of mocap
x_target.R(:,:,1) = Rwb(:,:,1);
x_target.v(:,1) = [0;0;0];
x_target.p(:,1) = twb(:,1);

% guess for R, initialize with ground truth of mocap
R_est(:,:,1) = x_target.R(:,:,1);
v_est(:,1) = x_target.v(:,1);
p_est(:,1) = x_target.p(:,1);

for i = 1:nkey-1
    j = i+1;

    [tij,omegaij,accelij] = keyframe_segmentation(imu,keyframe(i),keyframe(j));

    [dtij,dRij,dvij,dpij] = relative_motion(tij,omegaij,accelij);
    R_est(:,:,j) = dRij*R_est(:,:,i);
    v_est(:,j) = R_est(:,:,i)*dvij+v_est(:,i)+g*sum(dtij);
    p_est(:,j) = R_est(:,:,i)*dpij+p_est(:,i)+v_est(:,i)*sum(dtij)+1/2*g*sum(dtij.^2);
    
    Sigmaij(:,:,j) = noise_propagation_ij(tij,omegaij,accelij,Sigmaij(:,:,i));

%     x_target.R(:,:,j) = R_est(:,:,j);
%     x_target.v(:,j) = v_est(:,j);
%     x_target.p(:,j) = p_est(:,j);
    x_target.R(:,:,j) = x_target.R(:,:,i);
    x_target.v(:,j) = x_target.v(:,i);
    x_target.p(:,j) = x_target.p(:,i);

    x_target = Gauss_Newton_Project(x_target,dtij,dRij,dvij,dpij,j,Sigmaij,g);

end

plot(p_est(1,:),p_est(2,:))
grid on;axis equal;hold on
plot(p_gt(1,:),p_gt(2,:))







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

R_est = zeros(size(R_gt));
R_est(:,:,1) = R_gt(:,:,1);
for i = 1:nkey-1
    j = i+1;
    [tij,omegaij,accelij] = keyframe_segmentation(imu,keyframe(i),keyframe(j));
    [dtij,dRij,dvij,dpij] = relative_motion(tij,omegaij,accelij);
    R_est(:,:,i+1) = R_est(:,:,i)*dRij;
end

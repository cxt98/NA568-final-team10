function [tij,omegaij,accelij] = keyframe_segmentation(imu,keyframei,keyframej)
idx = keyframei:keyframej;                  % 1 by 11
tij = imu.time(idx);                        % 1 by 11
omegaij = imu.omega(:,idx);                 % 3 by 11   
accelij = imu.accel(:,idx);                 % 3 by 11
end


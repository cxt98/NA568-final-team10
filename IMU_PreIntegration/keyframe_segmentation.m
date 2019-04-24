function [tij,omegaij,accelij] = keyframe_segmentation(imu,keyframei,keyframej)
idx = keyframei:keyframej;                  % 1 by 11
tij = imu.time(idx);                        % 1 by 11
omegaij = imu.angularVelocity(:,idx);                 % 3 by 11   
accelij = imu.linearAcceleration(:,idx);                 % 3 by 11
end


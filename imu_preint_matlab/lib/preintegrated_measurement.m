function [dR_ij, dv_ij, dp_ij] = preintegrated_measurement(dt_ij, w_ij, ...
    a_ij, bg_i, ba_i)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Generates the relative motion
    % increments used to calculate the IMU factors between two keyframes,
    % at timsteps i and j. This corresponds to equations (33) and (35) -
    % (38) in Forster, et al. (2016).
    %
    % Inputs:
    %   dt_ij       Time intervals at timesteps between i and j,
    %               according to IMU timekeeping, each R^1
    %   w_ij        Gyro measurements between i and j-1, each R^3
    %   a_ij        Gyro measurements between i and j-1, each R^3
    %   bg_i        Gyro bias at first keyframe, R^3
    %   ba_i        Accelerometer bias at first keyframe, R^3
    %
    % Outputs:
    %   dR_ij       Relative rotation matrix estimate of body frame w.r.t.
    %               world frame at j (relative to i), SO(3)
    %   dv_ij     	Relative world-frame velocity estimate at j (relative
    %               to i), R^3 
    %   dp_ij     	Relative world-frame position estimate at j (relative
    %               to i), R^3 
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190413
    
    % Calculate relative measurements unsing iterative model
    % Initialize to trivial quantities on SO(3), R^3
    dR_ij = eye(3);
    dv_ij = zeros(3,1);
    dp_ij = zeros(3,1);
    
    % Loop through measurements to build relative motion quantities
    n_ts = length(dt_ij);
    for kk = 1:n_ts
        [dR_ij, dv_ij, dp_ij] = imu_model_kf_delta_iterative(w_ij(kk), ...
            a_ij(kk), dR_ij, dv_ij, dp_ij, bg_i, ba_i, dt_ij(kk));
    end
end

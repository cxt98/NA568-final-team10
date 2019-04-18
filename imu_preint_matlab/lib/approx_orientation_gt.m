function [R_gt, q_gt] = approx_orientation_gt(t_gt, t_imu, q_imu)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Approximates the ground truth
    % orientation. Data for the Ross Hartley dataset by:
    %   1. Finding the IMU timestep closest to each "GT" (MoCap) timestep
    %   2. Get the corresponding orientation quaternion to that IMU
    %   timestep.
    %   3. Calculate and store a rotation matrix from that quaternion.
    %
    % Inputs:
    %   t_gt        Ground truth (MoCap) time instances
    %   t_imu       IMU time instances
    %   q_imu       IMU orientation quaternions
    %
    % Outputs:
    %   R_gt        "Ground truth" orientations (rotation matrices)
    %   q_gt        "Ground truth" orientations (vectorized quaternions)
    %               This one is mostly included for reference since we
    %               don't use a quaternion formulation in our SLAM
    %               implementation.
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190413
    
    % Setup ranging parameters
    kk = 1;
    n_gt = length(t_gt);
    
    % Preallocate/setup data structures
    q_gt = zeros(4,n_gt);
    R_gt = {}; % Choose to use cell array, working with list of matrices
    
    % Iterate through all time instances in the ground truth
    for ii = 1:n_gt
        % Iterate through the time instances in the IMU
        while t_imu(kk) < t_gt(ii)
            kk = kk + 1;
        end
        
        % Now we have a IMU time instance kk that is greater than or equal
        % to the ground truth time instance ii.
        if t_imu(kk) == t_gt(ii)  % Same exact time = easy decision
            q_gt(:,ii) = q_imu(:,kk); % Also captures t = 0 corner case
        elseif t_imu(kk) > t_gt(ii)
            % Get the changes in time between the IMU timestep just past
            % the GT timestep, and between the GT timestep and the IMU
            % timestep just before. Whichever is smaller, use that
            % quaternion.
            d_later = t_imu(kk) - t_gt(ii);
            d_sooner = t_gt(ii) - t_imu(kk-1);
            
            if d_later <= d_sooner
                q_gt(:,ii) = q_imu(:,kk);
            else
                q_gt(:,ii) = q_imu(:,kk-1);
            end
        else % This should never occur, hence error.
            assert(t_imu(kk) >= t_gt(ii), 'Time comparison weirdness!');
        end
        
        % Now get the rotation matrix.
        % Normalize the quaternion to a unit quaternion, such that we
        % convert to a rotation matrix on SO(3) - data is *close* to unit
        % quaternions, but we can improve somewhat.
        q_gt(:,ii) = normalize_quaternion(q_gt(:,ii));
        
        % Final transpose to make MATLAB's formulation match ours
        % ^^ Check with Joe for validity of above comment ^^
        R_ii = quat2rotm(q_gt(:,ii)');
        
        % Validate rotation matrix and store - would be nice if we could
        % "normalize" to SO(3) here, if such a method exists
        check_SO3(R_ii);
        R_gt(1,ii) = {R_ii};
    end
end

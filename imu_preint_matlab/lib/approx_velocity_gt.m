function v_gt = approx_velocity_gt(t_gt, p_gt)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Approximates the ground truth
    % velocity data for the Ross Hartley dataset by dividing the change in
    % position over the previous time increment by the length of that time
    % increment. The exception to this is the first timestep: we assume the
    % robot starts from rest, so we assign the velocity at the first time
    % instance to be identically 0.
    %
    % Inputs:
    %   t_gt        Ground truth (MoCap) time instances
    %   p_gt        Ground truth positions
    %
    % Outputs:
    %   v_gt        "Ground truth" velocities
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190413
    
    % Preallocate
    n_gt = length(t_gt);
    v_gt = zeros(3,n_gt); % Implicitly sets initial velocity to 0
    
    % Approximate velocities
    for ii = 2:n_gt
        dp_ij = p_gt(:,ii) - p_gt(:,ii-1);
        dt_ij = t_gt(ii) - t_gt(ii-1);
        v_gt(:,ii) = dp_ij ./ dt_ij;
    end
end

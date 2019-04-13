function [dt_ij, w_ij, a_ij] = isolate_increment(ti, tj, t_imu, w_imu, ...
    a_imu)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Determines which timesteps by IMU
    % timekeeping fall between two timesteps by Visual timekeeping. Returns
    % the corresponding time increments and gyro and accelerometer
    % sub-lists. ASSUMPTION: that IMU timekeeping timesteps have 1:1
    % correspondence with IMU measurements (gyro and accelerometer).
    %
    % Inputs:
    %   ti          Time at timestep i (first keyframe), by visual
    %               timekeeping 
    %   tj          Time at timestep j (second keyframe), by visual
    %               timekeeping
    %   t_imu       Times at IMU timekeeping timesteps
    %   w_imu       Gyro measurements at IMU timekeeping timesteps
    %   a_imu       Accelerometer measurements at IMU timekeeping timesteps
    %
    % Outputs:
    %   dt_ij       Delta-t time increment (IMU timekeeping) between times 
    %               i (inclusive) and j (inclusive)
    %   w_ij        Gyro measurements between times i (incl.) and j (excl.)
    %   a_ij        Accelerometer measurements between times i (incl.) and
    %               j (excl.)
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190413
    
    % Iterate through time list to get bookend elements
    % We assume that ti = t_imu(ii) and tj = t_imu(jj).
    kk = 1;
    while t_imu(kk) < ti
        kk = kk + 1;
    end
    ii = kk;
    while t_imu(kk) <= tj
        kk = kk + 1;
    end
    jj = kk;
    
    % Get all time instances
    t_ij = t_imu(1,ii:jj);
    % Get all measurement instances (instance j-exclusive)
    w_ij = w_imu(3,ii:jj-1);
    a_ij = a_imu(3,ii:jj-1);
    
    % Calculate time increments which start at time instances i through j-1
    dt_ij = zeros(1,length(t_ij)-1);
    for kk = 1:n_ts
        dt_ij(kk) = t_ij(k+1) - t_ij(k);
    end
end

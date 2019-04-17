function [R_k, v_k, p_k] = imu_model_integration(w_k, a_k, R_km1, ...
      v_km1, p_km1, bg_km1, ba_km1, g, dt)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Updates the motion model of the IMU,
    % integrating its kinematics to obtain its state at the following
    % timestep. This corresponds to equation (31) in Forster, et al. 
    % (2016).
    %
    % Inputs:
    %   w_k         Body-frame angular velocity IMU measurement at current
    %               timestep, R^3
    %   a_k         Body-frame acceleration IMU measurement at current
    %               timestep, R^3
    %   R_km1       Rotation matrix estimate of body frame w.r.t. world
    %               frame at previous timestep k-1, SO(3)
    %   v_km1       World-frame velocity estimate at previous timestep, R^3
    %   p_km1       World-frame position estimate at previous timestep, R^3
    %   bg_km1      Gyro bias at previous timestep, R^3
    %   ba_km1      Accelerometer bias at previous timestep, R^3
    %   g           World-frame gravitational acceleration, [0 0 -9.81]
    %   dt          Change in time between timesteps k-1 and k, R^1
    %
    % Outputs:
    %   R_k         Rotation matrix estimate of body frame w.r.t. world
    %               frame at current timestep k-1, SO(3)
    %   v_k      	World-frame velocity estimate at current timestep, R^3 
    %   p_k      	World-frame position estimate at current timestep, R^3
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190406
    
    R_k = R_km1 * exp_map((w_k - bg_km1)*dt,'rodrigues');
    v_k = v_km1 + g*dt + R_km1*(a_k - ba_km1)*dt;
    p_k = p_km1 + v_km1*dt + 0.5*g*dt^2 + 0.5*R_km1*(a_k - ba_km1)*dt^2;
end

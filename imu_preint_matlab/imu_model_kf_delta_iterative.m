function [dR_ik, dv_ik, dp_ik] = imu_model_kf_delta_iterative(w_k, a_k, ...
    dR_ikm1, dv_ikm1, dp_ikm1, bg_i, ba_i, dt)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Updates the relative motion increments
    % used to calculate the IMU factors with each timestep k from previous
    % timestep k-1 ("km1"). Should be called in a loop over the IMU
    % measurements between two keyframes to build the appropriate relative
    % motion increments. This corresponds to equations (33) and (35) - (38)
    % in Forster, et al. (2016).
    %
    % Inputs:
    %   w_k         Body-frame angular velocity IMU measurement at current
    %               timestep k, R^3
    %   a_k         Body-frame acceleration IMU measurement at current
    %               timestep k, R^3
    %   dR_ikm1     Relative rotation matrix estimate of body frame w.r.t.
    %               world frame at previous timestep k-1, SO(3)
    %   dv_ikm1     Relative world-frame velocity estimate at k-1, R^3
    %   dp_ikm1     Relative world-frame position estimate at k-1, R^3
    %   bg_i        Gyro bias at previous keyframe, R^3
    %   ba_i        Accelerometer bias at previous keyframe, R^3
    %   dt          Change in time between timesteps k-1 and k, R^1
    %
    % Outputs:
    %   dR_ik       Relative rotation matrix estimate of body frame w.r.t.
    %               world frame at current timestep k, SO(3)
    %   dv_ik     	Relative world-frame velocity estimate at k, R^3 
    %   dp_ik     	Relative world-frame position estimate at k, R^3
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190413
    
    % dRik = Ri'*Rk = PROD(k=i:j-1)[exp_map((wk - bgk - ngk)*dtk)]
    dR_ik = dR_ikm1 * exp_map((w_k - bg_i)*dt,'rodrigues');
    
    % dvij = vi + g*SUM(k=i:j-1)[dtk] + SUM(k=i:j-1)[Rk * (ak - bak -
    % nak)*dtk]
    dv_ik = dv_ikm1 + dR_ik * (a_k - ba_i) * dt;
    
    % pj = pi + SUM(k=i:j-1)[vk*dtk + 0.5*g*dtk + 0.5*Rk(ak - bak - nak) * 
    % dtk^2]
    dp_ik = dp_ikm1 + dv_ik * dt + 0.5 * dR_ik * (a_k - ba_i) * dt^2;
end
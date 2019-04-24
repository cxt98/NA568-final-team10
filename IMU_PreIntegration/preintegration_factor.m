function [r_ij, J_ij] = preintegration_factor(R_i, R_j, v_i, v_j, p_i, ...
    p_j, dR_ij, dv_ij, dp_ij, dt_ij, g)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Generates the residual and the
    % jacobianfor the IMU factor based on the relative motion between
    % keyframes i and j. This relies on the preintegrated IMU measurement,
    % as well as estimates of the vehicle postion, velocity, and
    % orientation at i and j. These estimates may be ground truth in the
    % IMU demo case, or decision variables when the IMU factor is used with
    % Graph SLAM. This corresponds to equation (45) and Appendix IX-C
    % (Equations (70) - (81)) in Forster, et al. (2016).
    %
    % Inputs:
    %   R_i         Rotation matrix estimate of body frame w.r.t. world
    %               frame at current timestep i, SO(3)
    %   R_j         Rotation matrix estimate of body frame w.r.t. world
    %               frame at current timestep j, SO(3)
    %   v_i         World-frame velocity estimate at i, R^3 
    %   v_j         World-frame velocity estimate at j, R^3 
    %   p_i         World-frame position estimate at i, R^3 
    %   p_j         World-frame position estimate at j, R^3 
    %   dR_ij       Relative rotation matrix estimate of body frame w.r.t.
    %               world frame at j (relative to i), SO(3)
    %   dv_ij       Relative world-frame velocity estimate at j (relative
    %               to i), R^3 
    %   dp_ij       Relative world-frame position estimate at j (relative
    %               to i), R^3 
    %   dt_ij       Time intervals at timesteps between i and j,
    %               according to IMU timekeeping, each R^1
    %   g           World-frame gravitational acceleration, [0 0 -9.81]
    %
    % Intermediate quantities:
    %   r_dRij      Relative rotation residual for an IMU factor between
    %               keyframes i and j, R^3
    %   r_dvij     	Relative velocity residual for an IMU factor between
    %               keyframes i and j, R^3
    %   r_dpij     	Relative position residual for an IMU factor between                                                                                           
    %               keyframes i and j, R^3
    %   J_dRij      Jacobian of r_dRij
    %   J_dvij      Jacobian of r_dvij
    %   J_dpij      Jacobian of r_dpij
    %
    % Outputs:
    %   r_ij        Stacked state residual for this factor
    %   J_ij        Stacked residual error Jacobian for this factor
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190416
    
    % Sum total time increment (IMU timekeeping)
    sdt_ij = sum(dt_ij);
    
    % Equation (45) - residual calculation
    r_dRij = so3_to_R3(logm((dR_ij)'*R_i'*R_j));
    
    r_dvij = R_i'*(v_j-v_i-g*sdt_ij)-(dv_ij);
    
    r_dpij = R_i'*(p_j-p_i-v_i*sdt_ij-0.5*g*sdt_ij^2)-(dp_ij);
    
    % Because of the rotation, the nonlinear solver uses a
    % Lift-Solve-Retract technique. These Jacobians are defined such that
    % the cost function has been lifted; retractions are already
    % substituted. See Equation (70) in Forster, et al. (2016).
    
    % d-vector formation: dphi_i, dv_i, dp_i, dphi_j, dv_j, dp_j
    
    % Jacobian of r_dpij (R^3x18):
    J_dpij = [R3_to_so3(R_i'*(p_j-p_i-v_i*sdt_ij-0.5*g*sdt_ij^2)) ...
              -R_i'*sdt_ij ... % dv_i                           % dphi_i
              -eye(3,3) ...    % dp_i
              zeros(3,3) ...   % dphi_j
              zeros(3,3) ...   % dv_j
              R_i'*R_j];       % dp_j
    % Jacobian of r_dvij (R^3x18):
    J_dvij = [R3_to_so3(R_i'*(v_j-v_i-g*sdt_ij)) ... % dphi_i
              -R_i' ...      % dv_i
              zeros(3,3) ... % dp_i
              zeros(3,3) ... % dphi_j
              R_i' ...       % dv_j
              zeros(3,3)];   % dp_j
      
    % Jacobian of r_dRij (R^3x18), Equation (81):
    J_dRij = [-SO3_InverseRightJacobian(r_dRij)*R_j'*R_i ... % dphi_i
              zeros(3,3) ... % dv_i
              zeros(3,3) ... % dp_i
              SO3_InverseRightJacobian(r_dRij) ... % dphi_j
              zeros(3,3) ... % dv_j
              zeros(3,3)];   % dp_j
          
%     J_dpij = [R3_to_so3(R_i'*(p_j-p_i-v_i*sdt_ij-0.5*g*sdt_ij^2)) ...% dphi_i
%               -eye(3,3) ...    % dp_i
%               zeros(3,3) ...   % dphi_j
%               R_i'*R_j];       % dp_j
%    
%       
%     % Jacobian of r_dRij (R^3x18), Equation (81):
%     J_dRij = [-SO3_InverseRightJacobian(r_dRij)*R_j'*R_i ... % dphi_i
%               zeros(3,3) ... % dp_i
%               SO3_InverseRightJacobian(r_dRij) ... % dphi_j
%               zeros(3,3)];   % dp_j
      
% % %     % Jacobian of r_dRij (R^3x18), Equation (81):
% % %     J_dRij = [-SO3_InverseRightJacobian(r_dRij)*R_j'*R_i ... % dphi_i
% % %               SO3_InverseRightJacobian(r_dRij)]; ... % dphi_j
          
    % Stack quantities
    r_ij = [r_dRij;r_dvij;r_dpij];
    J_ij = [J_dRij;J_dvij;J_dpij];
end

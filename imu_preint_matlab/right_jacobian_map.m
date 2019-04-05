function phi_RJSO3 = right_jacobian_map(phi_R3)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Maps an R3 vector (which corresponds
    % to an so(3) representation) to a matrix representing the right
    % Jacobian of SO(3). This corresponds to equation (8) in Forster, et
    % al. (2016).
    %
    % Inputs:
    %   phi_R3      vector in R^3
    %
    % Outputs:
    %   phi_RJSO3   3x3 matrix, right Jacobian of SO(3)
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190405
    
    % Check if in R^3
    check_R3(phi_R3);
    
    % Convert R^3 representation to so(3)
    phi_so3 = vector_to_so3(phi_R3);
    
    % Create norm of phi
    phi_norm = norm(phi_R3,2);

    % Complete right Jacobian formula
    phi_RJSO3 = eye(3) - (1-cos(phi_norm))/phi_norm^2 * phi_so3 + ...
        (phi_norm-sin(phi_norm))/phi_norm^3 * phi_so3^2;  
end

function omega_R3 = so3_to_vector(omega_so3)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Converts a matrix in so(3) to a vector
    % representation in R^3. This corresponds to equation (1) in Forster,
    % et al. (2016).
    %
    % Inputs:
    %   omega_so3   skew-symmetric R^3x3 matrix
    %
    % Outputs:
    %   omega_R3    vector in R^3
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190405
    
    % Check that matrix is in fact in so(3)
    check_laso3(omega_so3);
    
    % Convert matrix
    omega_R3 = [omega_so3(3,2) omega_so3(1,3) omega_so3(2,1)]';     
end

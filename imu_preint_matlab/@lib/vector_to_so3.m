function omega_so3 = vector_to_so3(omega_R3)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Converts a 3-vector representation to
    % a 3x3 skew symmetric representation, mapping a vector to so(3), the
    % Lie algebra of SO(3). This corresponds to equation (1) in Forster, et
    % al. (2016).
    % 
    % Inputs:
    %   omega_R3    vector in R^3
    % 
    % Outputs:
    %   omega_so3   skew-symmetric R^3x3 matrix
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190405
    
    % Check if in R^3
    check_R3(omega_R3);
    
    % Convert
    omega_so3 = [0 -omega_R3(3) omega_R3(2);
                 omega_R3(3) 0 -omega_R3(1);
                 -omega_R3(2) omega_R3(1) 0];    
end

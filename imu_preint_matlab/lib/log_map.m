function phi_R3 = log_map(phi_SO3)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Maps an SO(3) rotation matrix  to an
    % R^3 vector (which corresponds to an so(3) representation). This
    % corresponds to equation (5) in Forster, et al. (2016).
    %
    % Inputs:
    %   phi_SO3     3x3 matrix on SO(3) (e.g. a DCM)
    %
    % Outputs:
    %   phi_R3      vector in R^3 (e.g. angular velocity)  
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190405
    
    % Check if in SO(3) and != I
    check_SO3(phi_SO3);
    % We could make a threshold check here, but during residual
    % calculation, this matrix will be very close to I_3x3 anyways. Since
    % we don't want to throw out any cases arbitrarily close to I, we make
    % this a hard not-equals.
    assert(all(phi_SO3 ~= eye(3),'all'), 'Input matrix may not equal I.');
    
    % Convert to so(3)
    phis = acos((trace(phi_SO3) - 1)/2);
    phi_so3 = phis * (phi_SO3 - phi_SO3') / (2 * sin(phis));
    
    % Convert to R^3
    phi_R3 = so3_to_vector(phi_so3);    
end

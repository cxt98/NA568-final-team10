function phi_SO3 = exp_map(phi_R3, map_type)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Maps an R3 vector (which corresponds
    % to an so(3) representation to an SO(3) rotation matrix. This
    % corresponds to equations (3) and (4) in Forster, et al. (2016).
    %
    % Inputs:
    %   phi_R3      vector in R^3 (e.g. angular velocity)
    %
    % Outputs:
    %   phi_SO3     3x3 matrix in SO(3) (e.g. a DCM)
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190405
    
    % Handle case where no map type is passed in
    if ~exist('map_type','var') || isempty(map_type)
        map_type = 'rodrigues';
    end
    
    % Check if in R^3
    check_R3(phi_R3);
    
    % Convert R^3 representation to so(3)
    phi_so3 = vector_to_so3(phi_R3);
    
    % Use either Rodrigues' Formula or first-order approximation
    switch lower(map_type)
        case 'rodrigues'
            % Create norm of phi
            phi_norm = norm(phi_R3,2);

            % Complete Rodrigues' Formula
            phi_SO3 = eye(3) + sin(phi_norm)/phi_norm * phi_so3 + ...
                (1-cos(phi_norm))/phi_norm^2 * phi_so3^2;  
        case 'first_order'
            phi_SO3 = eye(3) + phi_so3;
        otherwise
            error('unrecognized mapping method: "%s"', map_type);
    end 
end

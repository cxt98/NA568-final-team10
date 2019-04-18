function params = get_imu_params(name)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Returns the IMU parameters
    % corresponding to certain datasets including Cassie Blue.
    %
    % Inputs:
    %   name        Name of dataset
    %
    % Outputs:
    %   v_gt        IMU parameter struct
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190417
    
    % Gravity is consistent across models
    params.g_mag = 9.80665;
    params.g = [0 0 -9.80665]';
    
    % Pick based on name
    switch name
        case 'cassie'
            % Source: 
            % https://www.vectornav.com/docs/default-source/documentation/
            % vn-100-documentation/PB-12-0002.pdf?sfvrsn=9f9fe6b9_18
            %
            % Consider checking numbers based on this:
            % https://www.vectornav.com/support/library/gyroscope
            params.rate = 2000;
            params.sigma_g = 0.0035*pi/180*sqrt(params.rate); % rad/s
            params.Sigma_g = params.sigma_g*eye(3);
            params.sigma_a = 0.14e-3*params.g_mag*sqrt(params.rate); %m/s^2
            params.Sigma_a = params.sigma_a*eye(3);
            params.sigma_bg = 10 * pi/180 * 1/3600; % rad/s
            params.Sigma_bg = params.sigma_bg * eye(3);
            params.sigma_ba = 0.4 * params.g_mag / 1000; % m/s^2
            params.Sigma_ba = params.sigma_ba * eye(3);
        otherwise
            error('unrecognized dataset name');
    end 
end

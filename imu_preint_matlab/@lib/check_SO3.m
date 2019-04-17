function check_SO3(m)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Checks whether a matrix meets the
    % requirements to be on so(3), and throws an error if it does not.
    %
    % Inputs:
    %   m       skew-symmetric R^3x3 matrix
    %
    % Outputs:
    %   none
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190405

    % Establish a numerical accuracy cutoff threshold
    eps = 1e-13;
    
    % Run checks
    sz = size(m);
    assert((sz(1) == 3 && sz(2) == 3),'Matrix not 3x3');    
    assert((det(m) - 1 < eps && all(m'*m - eye(3) < eps,'all') && ... 
            all(m*m' - eye(3) < eps,'all')), 'Matrix not in SO(3)');
end

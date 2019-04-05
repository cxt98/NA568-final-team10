function check_laso3(m)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Checks whether a matrix meets the
    % requirements to be in so(3) (Lie algebra of SO(3)), and throws an
    % error if it does not.
    %
    % Inputs:
    %   m       skew-symmetric R^3x3 matrix
    %
    % Outputs:
    %   none
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190405

    sz = size(m);
    assert((sz(1) == 3 && sz(2) == 3),'Matrix not 3x3');    
    assert((m(1,1) == 0 && m(2,2) == 0 && ...
            m(3,3) == 0 && m(2,1) == -m(1,2) && ...
            -m(3,1) == m(1,3) && ...
            m(3,2) == -m(2,3)), ...
            'Matrix not in so(3)');
end

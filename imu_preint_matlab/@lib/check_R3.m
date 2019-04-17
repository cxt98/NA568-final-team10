function check_R3(v)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Checks whether a vector meets the
    % requirements to be in R^3, and throws an error if it does not.
    %
    % Inputs:
    %   v       R^3 vector
    %
    % Outputs:
    %   none
    %
    % Contributors: Joseph Yates
    % Last Edited: 20190405

    sz = size(v);
    assert((sz(1) == 3 && sz(2) == 1),'Vector not 3x1');
end

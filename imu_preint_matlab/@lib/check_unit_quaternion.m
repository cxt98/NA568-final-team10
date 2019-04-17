function check_unit_quaternion(q)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Checks whether a vector meets the
    % requirements to be a unit quaternion, and throws an error if it does
    % not.
    %
    % Inputs:
    %   q       unit quaternion represented as a 4-vector
    %
    % Outputs:
    %   none
    %
    % Contributors: Joseph Yates
    % Last Edited: 201904013
    
    % Establish a numerical accuracy cutoff threshold
    eps = 1e-10;
    
    % Run checks
    sz = size(q);
    assert((sz(1) == 4 && sz(2) == 1),'Vector not 4x1');    
    assert(all(norm(q,2) - 1 < eps,'all'), 'Quaternion not unitary');
end

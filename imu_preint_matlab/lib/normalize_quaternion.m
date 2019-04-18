function qu = normalize_quaternion(q)
    % NAVARCH 568 W19 Group 10
    % Chen, Dai, Lu, Yates
    % Final Project: IMU Preintegration
    %
    % Function: call with arguments. Normalizes a quaternion to return a
    % corresponding unit quaternion. Assumes the input q is in fact a
    % quaternion (e.g. a 4-vector), check this independently beforehand if
    % this assumption may not hold due to other code.
    %
    % Inputs:
    %   q       arbitrary quaternion represented as a 4-vector
    %
    % Outputs:
    %   qu      unit quaternion represented as a 4-vector
    %
    % Contributors: Joseph Yates
    % Last Edited: 201904013
    qu = q ./ norm(q,2);    
end

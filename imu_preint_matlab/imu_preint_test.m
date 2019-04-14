% NAVARCH 568 W19 Group 10
% Chen, Dai, Lu, Yates
% Final Project: IMU Preintegration
%
% Script: Run this in its current directory using MATLAB.
%
% Contributors: Joseph Yates
% Last Edited: 20190413

% Housekeeping
clear all;
close all;

% Load Dataset
load('test_data/mocap_vectorNav_data.mat');

% Global parameters
g = [0 0 -9.80665]'; % Gravity (1 g)

% We use MoCap position and time data to calculate "ground truth" for
% preintegrated position and velocity. We use the IMU orientation data for
% "ground truth" for the rotation matrix. See approx_velocity_gt.m and
% approx_orientation_gt.m for details.
v_gt = approx_velocity_gt(mocap.time, mocap.position);
[R_gt,~] = approx_orientation_gt(mocap.time, imu.time, imu.orientation);

% Bias modeling: currently unimplemented, not sure yet how to handle here
% since we're not filtering in this demo
bg = zeros(3,1);
ba = zeros(3,1);

% We have slightly more IMU data than MoCap data, so using MoCap as the
% "keyframes" should work fine. Preallocate quantities.
n_tm = length(mocap.time)-1; % number of increment's we'll work with
res_dRij = zeros(3,n_tm);    % size of residual data we'll end up with
res_dvij = zeros(3,n_tm);
res_dpij = zeros(3,n_tm);

% Iterate through mocap/ground truth points, which we'll use as keyframes
for ii = 1:n_tm  % Set i
    jj = ii + 1; % Set j - both ends of the increment
    
    % Pull out the IMU data that we're using over this increment
    [dt_ij, w_ij, a_ij] = isolate_increment(mocap.time(ii), ...
        mocap.time(jj), imu.time, imu.angularVelocity, ...
        imu.linearAcceleration);
    
%     % Temporary checks
%     for mm = 1:length(dt_ij)
%         check_R3(w_ij(:,mm));
%         check_R3(a_ij(:,mm));
%     end
%     disp('Measurements good!');
    
    % Bias modelling: not yet implemented
    bg_i = bg;
    ba_i = ba;
    
    % Calculate preintegrated measurements
    [dR_ij, dv_ij, dp_ij] = preintegrated_measurement(dt_ij, w_ij, ...
        a_ij, bg_i, ba_i);
    
%     % Temporary checks
%     check_SO3(dR_ij);
%     check_R3(dv_ij);
%     check_R3(dp_ij);
%     disp('Relative motion good!');
    
    % Calculate residuals
    [r_dRij, r_dvij, r_dpij] = preintegrated_residual(R_gt{ii}, ...
        R_gt{jj}, v_gt(ii), v_gt(jj), mocap.position(ii), ...
        mocap.position(jj), dR_ij, dv_ij, dp_ij, dt_ij, g);
    
    % Store residuals
    res_dRij(:,ii) = r_dRij;
    res_dvij(:,ii) = r_dvij;
    res_dpij(:,ii) = r_dpij;
end

% Plot residuals over time, so we see what we get
% Take norms of residuals
nresR = vecnorm(res_dRij);
nresv = vecnorm(res_dvij);
nresp = vecnorm(res_dpij);

% Pre-append 0: these were calculated on the j time instances, not the i's
nresR = [0 nresR];
nresv = [0 nresv];
nresp = [0 nresp];

% Plot against MoCap "GT" or "Visual" time
figure(1)
semilogy(mocap.time,nresR,mocap.time,nresv,mocap.time,nresp)
xlabel('time [s]');
ylabel('normed residual');
legend('R','v','p','Location','Southeast');

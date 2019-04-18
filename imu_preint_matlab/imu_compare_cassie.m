% NAVARCH 568 W19 Group 10
% Chen, Dai, Lu, Yates
% Final Project: IMU Preintegration
%
% Script: Run this in its current directory using MATLAB. Solves the
% nonlinear GN solution over IMU factors for Cassie Blue data using our
% implementation and using a GTSAM implementation. Compares our results,
% GTSAM results, and the ground truth. The GTSAM portion of this code is
% heavily based on the IMUKittiExampleGPS.m MATLAB included in the GTSAM
% MATLAB toolbox.
%
% Contributors: Joseph Yates
% Last Edited: 20190418

%% Setup
disp('-- Setting up environment');

% Housekeeping
clear all;
close all;
import gtsam.*;
addpath('lib');

% Load Dataset
load('test_data/mocap_vectorNav_data.mat');

% Global parameters
params = get_imu_params('cassie');
g = params.g; % Gravity (1 g)

%% Ground Truth
disp('-- Gathering ground truth data');

% Build struct
gt.t = mocap.time;
gt.p = mocap.position;

% We use MoCap position and time data to calculate "ground truth" for
% preintegrated position and velocity. We use the IMU orientation data for
% "ground truth" for the rotation matrix. See approx_velocity_gt.m and
% approx_orientation_gt.m for details.
gt.v = approx_velocity_gt(mocap.time, mocap.position);
[gt.R,~] = approx_orientation_gt(mocap.time, imu.time, imu.orientation);

% We have more IMU data than MoCap data, so using MoCap as the
% "keyframes" should work fine. For this proof-of-concept test, we'll use
% every 10th GT data point time as the keyframe time. 
gt_inc = 1;
gt.i = 1:gt_inc:length(mocap.time)-1;
n_kf = length(gt.i); % number of keyframes
gt.dt = 0.01; % approximated/hard-coded average time between keyframes

% Plot ground truth
figure(1)
plot3(gt.p(1,:),gt.p(2,:),gt.p(3,:),'-b');
axis equal
hold on;

figure(2)
plot(gt.p(1,:),gt.p(2,:),'-b');
axis equal
hold on;


%% GTSAM Implementation ("Baseline")
disp('-- Setting up GTSAM solution')

% Interpret metadata and compute relative sensor pose transforms
IMU_metadata = [0 0 0 0 0 0 params.sigma_a params.sigma_g 0 ...
    params.sigma_ba params.sigma_bg 1/params.rate];
meta_headers = {'BodyPtx' 'BodyPty' 'BodyPtz' 'BodyPrx' 'BodyPry' ...
    'BodyPrz' 'AccelerometerSigma' 'GyroscopeSigma' 'IntegrationSigma' ...
    'AccelerometerBiasSigma' 'GyroscopeBiasSigma' 'AverageDeltaT'};
IMU_metadata = cell2struct(num2cell(IMU_metadata), meta_headers, 2);
IMUinBody = Pose3.Expmap([IMU_metadata.BodyPtx; IMU_metadata.BodyPty;
    IMU_metadata.BodyPtz; IMU_metadata.BodyPrx; IMU_metadata.BodyPry;
    IMU_metadata.BodyPrz]);

% Interpret data
dt = zeros(size(imu.time));
dt(1) = imu.time(1);
for ii = 2:length(imu.time)
    dt(ii) = imu.time(ii) - imu.time(ii-1);
end
IMU_data_mat = [imu.time' dt' imu.linearAcceleration'  ...
    imu.angularVelocity'];
data_headers = {'Time' 'dt' 'accelX' 'accelY' 'accelZ' 'omegaX' ...
    'omegaY' 'omegaZ'};
IMU_data = cell2struct(num2cell(IMU_data_mat), data_headers, 2);

% Get initial conditions
% Use truth data to pin initial conditions, initially stationary
currentPoseGlobal = Pose3(Rot3(gt.R{1}), Point3(gt.p(:,1)));
currentVelocityGlobal = LieVector(gt.v(:,1));
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
% Consider changing this noise model to Gaussian
sigma_init_x = noiseModel.Isotropic.Precisions([0.0; 0.0; 0.0; 1; 1; 1]);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([0.100; 0.100; 0.100; ...
    5.00e-05; 5.00e-05; 5.00e-05]);
sigma_between_b = [IMU_metadata.AccelerometerBiasSigma * ones(3,1); ...
    IMU_metadata.GyroscopeBiasSigma * ones(3,1)];
w_coriolis = [0;0;0];

% Solver object
isamParams = ISAM2Params;
isamParams.setFactorization('QR');
isamParams.setRelinearizeSkip(10);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;

% Data collection (in a not-GTSAM format)
bl.t = zeros(1, length(gt.i));
bl.p = zeros(3, length(gt.i));
bl.v = zeros(3, length(gt.i));
bl.ba = zeros(3, length(gt.i));
bl.bg = zeros(3, length(gt.i));
bl.R = {};

% Solution loop
disp('-- Starting GTSAM solution loop')
IMUtimes = [IMU_data.Time];

for ii = gt.i
  disp(['Iteration: ' num2str(ii)]);
  % At each "keyframe" we initialize a new node in the graph (use truth
  % data "keyframe" time
  currentPoseKey = symbol('x',ii);
  currentVelKey =  symbol('v',ii);
  currentBiasKey = symbol('b',ii);
  t = gt.t(ii);
  
  if ii == 1
    % Create initial prior on initial pose, velocity, and biases
    newValues.insert(currentPoseKey, currentPoseGlobal);
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);
    newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, ...
        sigma_init_x));
    newFactors.add(PriorFactorLieVector(currentVelKey, ...
        currentVelocityGlobal, sigma_init_v));
    newFactors.add(PriorFactorConstantBias(currentBiasKey, currentBias, ...
        sigma_init_b));
    
    % Store data
    bl.t(1,ii) = t;
    bl.R(1,ii) = {currentPoseGlobal.rotation().matrix()};
    bl.p(:,ii) = currentPoseGlobal.translation().vector();
%     bl.v(:,ii) = currentVelocityGlobal.vector();
%     bothbias = currentBias.vector();
%     bl.ba(:,ii) = bothbias(1:3);
%     bl.bg(:,ii) = bothbias(4:6);
  else
    % Summarize IMU data between the previous and current "keyframe"
    t_previous = gt.t(ii-gt_inc);
    IMUindices = find(IMUtimes >= t_previous & IMUtimes <= t);
    
    currentSummarizedMeasurement = ...
        gtsam.ImuFactorPreintegratedMeasurements(currentBias, ...
        IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
        IMU_metadata.GyroscopeSigma.^2 * eye(3), ...
        IMU_metadata.IntegrationSigma.^2 * eye(3));
    
    for imuIndex = IMUindices
        accMeas = [IMU_data(imuIndex).accelX; ...
            IMU_data(imuIndex).accelY; IMU_data(imuIndex).accelZ];
        omegaMeas = [IMU_data(imuIndex).omegaX; ...
            IMU_data(imuIndex).omegaY; IMU_data(imuIndex).omegaZ];
        deltaT = IMU_data(imuIndex).dt;
        currentSummarizedMeasurement.integrateMeasurement(accMeas, ...
            omegaMeas, deltaT);
    end
    
    % Create IMU factor
    newFactors.add(ImuFactor( ...
        currentPoseKey-gt_inc, currentVelKey-gt_inc, currentPoseKey, ...
        currentVelKey, currentBiasKey, currentSummarizedMeasurement, ...
        g, w_coriolis));
    
    % Bias evolution as given in the IMU metadata
    newFactors.add(BetweenFactorConstantBias(currentBiasKey-gt_inc, ...
        currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
        noiseModel.Diagonal.Sigmas(sqrt(numel(IMUindices)) * ...
        sigma_between_b)));

    % Add initial value
    newValues.insert(currentPoseKey, currentPoseGlobal);
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);
    
    % Update solver
    % =====================================================================
    % We accumulate 3 total keyframes before updating the solver
    % at first so that the heading becomes observable.
%     if ii > 10 % firstGPSPose + 2*GPSskip
        isam.update(newFactors, newValues);
        newFactors = NonlinearFactorGraph;
        newValues = Values;
      
%         if rem(ii,100)==1 % plot every 10 "keyframes"
          if ii == gt.i(end)
%             cla;
            figure(1)
            hold on;
            plot3DTrajectory(isam.calculateEstimate, 'g-');
%             title('Estimated trajectory using ISAM2 (IMU+GPS)')
%             xlabel('[m]')
%             ylabel('[m]')
%             zlabel('[m]')
            axis equal
            drawnow;
            
            figure(2)
            hold on;
            plot2DTrajectory(isam.calculateEstimate,'g-')
            axis equal
            drawnow;
          end
        % =================================================================
        currentPoseGlobal = isam.calculateEstimate(currentPoseKey);
        currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
        currentBias = isam.calculateEstimate(currentBiasKey);
        
        % Store data
        bl.t(1,ii) = t;
        bl.R(1,ii) = {currentPoseGlobal.rotation().matrix()};
        bl.p(:,ii) = currentPoseGlobal.translation().vector();
%         bl.v(:,ii) = currentVelocityGlobal.vector();
%         bothbias = currentBias.vector();
%         bl.ba(:,ii) = bothbias(1:3);
%         bl.bg(:,ii) = bothbias(4:6);
%     end
  end
end % end main loop

% Plot results
% figure(1)
% plot3(bl.p(1,:),bl.p(2,:),bl.p(3,:),'-g');
% hold on;
% 
% figure(2)
% plot(bl.p(1,:),bl.p(2,:),'-g');
% hold on;

%% Our implementation


% NAVARCH 568 W19 Group 10
% Chen, Dai, Lu, Yates
% Final Project: IMU Preintegration
%
% Script: Run this in its current directory using MATLAB.
%
% Contributors: Yizhou Lu, Joseph Yates
% Last Edited: 20190413

clc;clear;close all

load('test_data/mocap_vectorNav_data.mat')

% imu is collected every 0.0005 second
% motion is captured every 0.01 second
figure(1)
plot(mocap.position(1,:),mocap.position(2,:))
axis equal;grid on;hold on

figure(2)
plot3(mocap.position(1,:),mocap.position(2,:),mocap.position(3,:))
axis equal;grid on;hold on
% for i = 1:100
%     quiver(mocap.position(1,i),mocap.position(2,i),0.05*cos(mocap.position(3,i)),0.05*sin(mocap.position(3,i)))
% end
% R = eye(3);
R = quat2rotm(imu.orientation(:,1)');
v = zeros(3,1);
% p = zeros(3,1);
p = mocap.position(:,1);
bg = zeros(3,1);
ba = zeros(3,1);
g = [0 0 -9.80665]';
dt = 0.0005;

Traj(:,1) = p;

for kk = 1:length(imu.time)
%     R = R*rotation_matrix(imu.angularVelocity(:,kk),dt);
%     v = v+R*imu.linearAcceleration(:,kk)*dt;
%     p = p+v*dt+0.5*R*imu.linearAcceleration(:,kk)*dt^2;
    
    [R, v, p] = imu_model_integration(imu.angularVelocity(:,kk), ... 
        imu.linearAcceleration(:,kk), R, v, p, bg, ba, g, dt);
    Traj(:,kk+1) = p;
end

figure(1)
plot(Traj(1,:),Traj(2,:))

figure(2)
plot3(Traj(1,:),Traj(2,:),Traj(3,:));
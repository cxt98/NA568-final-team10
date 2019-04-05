clc;clear;close all

load('mocap_vectorNav_data.mat')

% imu is collected every 0.0005 second
% motion is captured every 0.01 second

plot(mocap.position(1,:),mocap.position(2,:))
axis equal;grid on;hold on
% for i = 1:100
%     quiver(mocap.position(1,i),mocap.position(2,i),0.05*cos(mocap.position(3,i)),0.05*sin(mocap.position(3,i)))
% end
R = eye(3);
v = zeros(3,1);
P = zeros(3,1);
dt = 0.0005;
for i = 1:length(imu.angularVelocity)
    R = rotation_matrix(imu.angularVelocity(:,i),dt);
    v = v+R*imu.linearAcceleration(:,i)*dt;
    P = P+v*dt+0.5*R*imu.linearAcceleration(:,i)*dt^2;
    Traj(:,i) = P;
end
plot(Traj(1,:),Traj(2,:))
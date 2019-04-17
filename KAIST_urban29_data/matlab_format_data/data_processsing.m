clear
global_p = csvread('global_pose.csv');
global_pose.timestamps = global_p(:,1) / 1e9;
for i = 1:length(global_pose.timestamps)
    T = [global_p(i,2:5); global_p(i,6:9); global_p(i,10:13); 0 0 0 1];
    global_pose.Transformation{i,1} = T;
end
save global_pose.mat global_pose

KF = load('KAIST_KeyFrame_Trajectory.txt');
Keyframe.timestamps = KF(:,1);
for i = 1:length(Keyframe.timestamps)
    Twc = [KF(i,2:5); KF(i,6:9); KF(i,10:13); 0 0 0 1];
    Keyframe.Twc{i,1} = Twc;
end
save Keyframe.mat Keyframe

Tvb = [eye(3), [-0.07; 0; 1.7]; 0 0 0 1];
Tvc = [-0.00413442 -0.0196634 0.999798 1.66944;
       -0.999931 -0.0109505 -0.00435034 0.278027;
        0.0110338 -0.999747 -0.0196168 1.61215;
        0 0 0 1];
Tcb = Tvc \ Tvb;
save Tcb.mat Tcb

xsens_imu = csvread('xsens_imu.csv');
imu.timestamps = xsens_imu(:,1) / 1e9;
for i = 1:length(imu.timestamps)
    imu.gyr{i,1} = xsens_imu(i,9:11);
    imu.acc{i,1} = xsens_imu(i,12:14);
end
save imu.mat imu


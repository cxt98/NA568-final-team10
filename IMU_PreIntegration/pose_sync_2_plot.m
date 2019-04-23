figure(2)
point_gt = zeros(3,length(T));
point_est = zeros(3,length(T));
for i = 1:length(T)
    point_gt(:,i) = T{i}(1:3,4);
    point_est(:,i) = T_est{i}(1:3,4);
end
plot3(point_gt(1,:),point_gt(2,:),point_gt(3,:))
hold on
plot3(point_est(1,:),point_est(2,:),point_est(3,:))
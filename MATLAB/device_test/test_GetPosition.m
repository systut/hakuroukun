clear
clc

gps_test_node = ros.Node("gps_test_node");
gps_subscriber = ros.Subscriber(gps_test_node,"/hakuroukun_pose/pose","geometry_msgs/PoseStamped");
imu_subscriber = ros.Subscriber(gps_test_node, "/hakuroukun_pose/orientation","std_msgs/Float64");

tic
while (toc < 1)
end

theta = GetTheta(imu_subscriber);
Pos = GetPosition(gps_subscriber, theta);

fprintf("Robot position : %f, %f - Robot Orientation %f \n", Pos(1), Pos(2), theta)
clear
clc

imu_test_node = ros.Node("imu_test_node");
imu_subscriber = ros.Subscriber(imu_test_node, "/hakuroukun_pose/orientation","std_msgs/Float64");

tic
while (toc < 1)
end

tic
while (toc < 50)
    theta = [imu_subscriber.LatestMessage.Data];
    fprintf("%f\n", theta);
end

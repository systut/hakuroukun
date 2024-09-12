clear
clc

gps_test_node = ros.Node("gps_test_node");
gps_subscriber = ros.Subscriber(gps_test_node,"/hakuroukun_pose/pose","geometry_msgs/PoseStamped");

tic
while (toc < 1)
end

while true
    x = gps_subscriber.LatestMessage.Pose.Position.X;
    y = gps_subscriber.LatestMessage.Pose.Position.Y;
    Pos = [x y];
    fprintf("Position : %f, %f \n", x, y);
end


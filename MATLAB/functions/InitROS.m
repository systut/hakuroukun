function [publisher, subscriber] = InitROS()
    % This function is used to initiate ros publisher and subscriber
    % Publisher : None
    % Subscriber : LiDAR(l/r) , IMU, GPS

    %Initiate pub&sub
    publisher = {};
    subscriber = {};

    % Device connection setup
    % ROS Node set up
    node = ros.Node('ros_to_matlab');

    % Sensor subscriber setup
    subscriber.right_lidar = ros.Subscriber(node,"/left_scan","sensor_msgs/LaserScan");
    subscriber.left_lidar = ros.Subscriber(node,"/right_scan","sensor_msgs/LaserScan");
    subscriber.imu = ros.Subscriber(node, "/hakuroukun_pose/orientation","std_msgs/Float64");
    subscriber.gps = ros.Subscriber(node, "/hakuroukun_pose/pose", "geometry_msgs/PoseStamped");
end

% ================
% Lidar Test Code
% ================

clear
clc
close all

% rosinit('127.0.0.1');
lidar_test_node = ros.Node("lidar_test_node");
left_lidar_sub = ros.Subscriber(lidar_test_node, "/left_scan", "sensor_msgs/LaserScan");
right_lidar_sub = ros.Subscriber(lidar_test_node, "/right_scan", "sensor_msgs/LaserScan");

test_time = 5;
tic;
while (toc<test_time)
    left_scan_msg = receive(left_lidar_sub,10);
    right_scan_msg = receive(right_lidar_sub,10);
    
    if (left_scan_msg.Ranges(1) < 0.3)
    end
end

left_scan= ReadLidar(left_scan_msg);
right_scan = ReadLidar(right_scan_msg);

% angle_min = -3.141592741012573;
% angle_max = 3.141592741012573;
% angle_increament = 0.003228769404814;
% angles = angle_min:angle_increament:angle_max;
% 
% left_scan = lidarScan(left_scan_msg.Ranges, angles);
% right_scan = lidarScan(right_scan_msg.Ranges, angles);
% 
% rotation_angle = -pi/2;
% right_x_robot = cos(rotation_angle)*right_scan.Cartesian(:,1) - sin(rotation_angle)*right_scan.Cartesian(:,2);
% right_y_robot = sin(rotation_angle)*right_scan.Cartesian(:,1) + cos(rotation_angle)*right_scan.Cartesian(:,2);
% right_scan_map = [right_x_robot, right_y_robot];
% 
% left_x_robot = cos(rotation_angle)*left_scan.Cartesian(:,1) - sin(rotation_angle)*left_scan.Cartesian(:,2);
% left_y_robot = sin(rotation_angle)*left_scan.Cartesian(:,1) + cos(rotation_angle)*left_scan.Cartesian(:,2);
% left_scan_map = [left_x_robot, left_y_robot];
% 
% phi = angles.' + pi;
% for i = 1:length(phi)
%     if (phi(i) > pi)
%         phi(i) = phi(i) - 2*pi;
%     end
% end
% 
% left_obstacle_odom = [phi, left_scan_msg.Ranges];
% right_obstacle_odom = [phi, right_scan_msg.Ranges];
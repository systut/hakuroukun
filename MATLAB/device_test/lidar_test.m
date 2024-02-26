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

left_scan_msg = receive(left_lidar_sub,10);
right_scan_msg = receive(right_lidar_sub,10);    

left_scan= ReadLidar(left_scan_msg);
right_scan = ReadLidar(right_scan_msg);
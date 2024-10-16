% TestInitROS.m
% This script tests the InitROS function

% Clear the workspace and initialize ROS
clear; clc;close all

% Initiate ROS 
[publisher, subscriber] = InitROS();

detect_status = 0;

[detect_status, scan_right] = LidarScan(subscriber.right_lidar, detect_status);
[detect_status, scan_left] = LidarScan(subscriber.left_lidar, detect_status);

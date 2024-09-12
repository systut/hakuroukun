% % ================
% % Lidar Test Code
% % ================
% 
% clear
% clc
% close all
% 
% robot_state = [0 0 pi/2 0 0];
% 
% % rosinit('127.0.0.1');
% lidar_test_node = ros.Node("lidar_test_node");
% left_lidar_sub = ros.Subscriber(lidar_test_node, "/left_scan", "sensor_msgs/LaserScan");
% right_lidar_sub = ros.Subscriber(lidar_test_node, "/right_scan", "sensor_msgs/LaserScan");
% 
% l_scan_msg = receive(left_lidar_sub, 10);
% r_scan_msg = receive(right_lidar_sub, 10);
% 
% [l_scan, flt_l_scan, l_f_scan, l_s_scan]= ReadLidar(l_scan_msg,"left");
% [r_scan, flt_r_scan, r_f_scan, r_s_scan] = ReadLidar(r_scan_msg,"right");

% [x_l, y_l, R_l] = ObstacleOdomLeft(robot_state, flt_l_scan);
% [x_r, y_r, R_r] = ObstacleOdomRight(robot_state, flt_r_scan);
% [x_ob, y_ob] = obodom(x_r,y_r,x_l,y_l,R_r,R_l);
detect_distance = 1.0;
for i=1:length(flt_r_scan)
    if (flt_r_scan(i,2) < detect_distance)
        detect = 1;
        break;
    end
end
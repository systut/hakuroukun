function [x_obst,y_obst,R_obst] = ObstacleOdomLeft(robot_state, scan_msg)
    % scan_msg = FilterScan(raw_scan_msg, "left");
    L1 = 0.33;     % Distance from GPS to Lidar
    x_lidar = robot_state(1) + L1*cos(robot_state(3)+deg2rad(70));
    y_lidar = robot_state(2) + L1*sin(robot_state(3)+deg2rad(70));
    
    distance = scan_msg(:,2);
    [~,idx] = min(distance);
    R_obst = scan_msg(idx,2);
    phi = scan_msg(idx,1);
    
    x_obst = R_obst*cos(robot_state(3)+phi) + x_lidar;
    y_obst = R_obst*sin(robot_state(3)+phi) + y_lidar;
end

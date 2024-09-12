function [scan_msg, filtered_scan_msg, front_scan_msg, side_scan_msg] = ReadLidar(raw_scan_msg, side)
    % Angle define
    angle_min = -3.141592741012573;
    angle_max = 3.141592741012573;
    angle_increament = 0.003228769404814;
    angles = angle_min:angle_increament:angle_max;
    rotation_angle = -pi/2;
    
    phi1 = 0:angle_increament:angle_max;
    phi2 = angle_min:angle_increament:-angle_increament;
    phi = [phi1 phi2];

    distance = raw_scan_msg.Ranges;

    scan = lidarScan(raw_scan_msg.Ranges, angles);
    x_robot = cos(rotation_angle)*scan.Cartesian(:,1) - sin(rotation_angle)*scan.Cartesian(:,2);
    y_robot = sin(rotation_angle)*scan.Cartesian(:,1) + cos(rotation_angle)*scan.Cartesian(:,2);
    scan_msg = [phi.', distance, x_robot, y_robot];
    
    % scan_msg = [phi.', distance];

    [filtered_scan_msg, front_scan_msg, side_scan_msg] = FilterScan(scan_msg, side);
end
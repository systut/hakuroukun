function [filtered_scan, front_scan, side_scan] = FilterScan(scan_msg, side)
    angle_increament = 0.003228769404814;
    max_idx = 1947;
    if isequal(side, "right")

        left_angle = deg2rad(45);
        right_angle = deg2rad(90);
        right_front_angle = deg2rad(45);

        left_idx = floor(left_angle/angle_increament);
        right_idx = max_idx - floor(right_angle/angle_increament);
        right_front_idx = max_idx - floor(right_front_angle/angle_increament);
        
        filtered_scan = [scan_msg(1:left_idx,:); scan_msg(right_idx:max_idx,:)];
        front_scan = [scan_msg(1:left_idx,:); scan_msg(right_front_idx:max_idx,:)];
        side_scan = scan_msg(right_idx:right_front_idx,:);

    elseif isequal(side, "left")

        right_angle = deg2rad(45);    
        left_angle = deg2rad(90);
        left_front_angle = deg2rad(45);

        left_idx = floor(left_angle/angle_increament);
        right_idx = max_idx - floor(right_angle/angle_increament);
        left_front_idx = floor(left_front_angle/angle_increament);
    
        filtered_scan = [scan_msg(1:left_idx,:); scan_msg(right_idx:max_idx,:)];
        front_scan = [scan_msg(1:left_front_idx,:); scan_msg(right_idx:max_idx,:)];
        side_scan = scan_msg(left_front_idx:left_idx,:);
    
    end
end
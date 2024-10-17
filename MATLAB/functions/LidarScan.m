function [detect_status, scanMsg] = LidarScan(lidar_sub, detect_status)
    % Scan parameters
    scanidx = 1;
    scanMsg = receive(lidar_sub,10);

    % Lidar spec
    angle_increament = rad2deg(0.004623388871551);
    scanidx_max = 1360;            %総データ数
    side_scan_angle = 30;          %判定除外角 
    front_scan_angle = 45;
    front_detect_distance = 1.0;    %閾値(長)
    side_detect_distance = 0.8;   %閾値(短)

    %% Begin scan
    % disp(getVarName(lidar_sub));
    if (lidar_sub.TopicName == "/right_scan")
        % disp(lidar_sub.TopicName)
        while(scanidx <= scanidx_max)
            if (((180+side_scan_angle)/angle_increament <= scanidx) && (scanidx < (345)/angle_increament))%側方検出範囲
                if (scanMsg.Ranges(scanidx) <= side_detect_distance && scanMsg.Ranges(scanidx)~= Inf)
                    detect_status = 1;
                end
            elseif ((0 < scanidx) && (scanidx <= (front_scan_angle)/angle_increament) || ((345)/angle_increament <= scanidx) && (scanidx < scanidx_max))%前方検出範囲
                if (scanMsg.Ranges(scanidx) <= front_detect_distance && scanMsg.Ranges(scanidx)~= Inf)
                    detect_status = 1;
                end
            end
            scanidx = scanidx+1;
        end
    end

    if (lidar_sub.TopicName == "/left_scan")
        % disp(lidar_sub.TopicName)
        while (scanidx <= scanidx_max)
            if (((15)/angle_increament < scanidx) && (scanidx <= (180-side_scan_angle)/angle_increament))%側方検出範囲
                if (scanMsg.Ranges(scanidx) <= side_detect_distance && scanMsg.Ranges(scanidx)~= Inf)
                    detect_status = 1;
                end
            elseif ((0 < scanidx)&&(scanidx <= (15)/angle_increament) || ((270+front_scan_angle)/angle_increament <= scanidx)&&(scanidx < scanidx_max))%前方検出範囲
                if (scanMsg.Ranges(scanidx) <= front_detect_distance && scanMsg.Ranges(scanidx)~= Inf)
                    detect_status = 1;
                end
            end
            scanidx = scanidx+1;
        end
    end
end

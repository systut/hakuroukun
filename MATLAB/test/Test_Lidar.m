clear all
format compact
n = 1;                  %点群データn番目
n_max = 720;            %総データ数
L_theta1 = 30;          %判定除外角 50 : arctan(39/33)
L_theta2 = 45;
distance_long = 1.7;    %閾値(長)
distance_long = 0.101;
distance_short = 0.5;   %閾値(短)
distance_short = 0.1;   %閾値(短)

rosinit('127.0.0.1');

node = ros.Node('rplidar_ros_to_matlab');
left_laser_sub = ros.Subscriber(node,"/left_scan","sensor_msgs/LaserScan");
right_laser_sub = ros.Subscriber(node,"/right_scan","sensor_msgs/LaserScan");
fprintf(" 1 \n");
while true
    
    fprintf(" 2 \n");

    % Obstacle detection check LiDAR left
    scanMsg2 = receive(left_laser_sub,10);
    n = 1;

    %検出工程2(Left)
    while(n <= n_max)
        if(((15)/0.5 < n) && (n <= (180-L_theta1)/0.5))%側方検出範囲
            if(scanMsg2.Ranges(n) <= distance_short)
                fprintf("Left Side obstacle detected\n");
            end
        elseif((0 < n)&&(n <= (15)/0.5) || ((270+L_theta2)/0.5 <= n)&&(n < n_max))%前方検出範囲
            if(scanMsg2.Ranges(n) <= distance_long )
                fprintf("Left Front obstacle detected\n");
            end
        end
        n = n+1;
    end

    fprintf(" 3 \n");
    % Obstacle detection check LiDAR right
    scanMsg1 = receive(right_laser_sub,10);
    n = 1;

    %検出工程1(Right)
    while(n <= n_max)
        if(((180+L_theta1)/0.5 <= n) && (n < (345)/0.5))%側方検出範囲
            if(scanMsg1.Ranges(n) <= distance_short)
                fprintf("Right Side obstacle detected\n");
            end
        elseif((0 < n)&&(n <= (L_theta2)/0.5) || ((345)/0.5 <= n)&&(n < n_max))%前方検出範囲
            if(scanMsg1.Ranges(n) <= distance_long )
                fprintf("Right Front obstacle detected\n");
            end
        end
        n = n+1;
    end
    fprintf(" 4 \n");   
end
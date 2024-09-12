%大学取材用（12/22）(1/17)
clc
clear
close all
format compact
%% Refactored parameters 
dt = 1.0;   %刻み時間[s]
global result; result.x = [];

%% Obstacle detection
global scanMsg1;        % Right Scan Message
global scanMsg2;        % Left Scan Message
global obstacle;        % Obstacle
angle_increament = rad2deg(0.004623388871551);
scan_max = 1360;            %総データ数
L_theta1 = 30;          %判定除外角 
L_theta2 = 45;
distance_long = 1.5;    %閾値(長)
distance_short = 0.75;   %閾値(短)
%%
DWA = 0;
DWA_go = 0;
velocity = 1;
count = 0;
com_s = 537;

%% Device connection setup
% LIDAR SETUP
% ノード設定
node = ros.Node('ros_to_matlab');
% LiDARデータのサブスクライバ
left_laser_sub = ros.Subscriber(node,"/left_scan","sensor_msgs/LaserScan");
right_laser_sub = ros.Subscriber(node,"/right_scan","sensor_msgs/LaserScan");
% Robot Pose Subscriber
imu_sub = ros.Subscriber(node, "/hakuroukun_pose/orientation","std_msgs/Float64");
gps_sub = ros.Subscriber(node, "/hakuroukun_pose/pose", "geometry_msgs/PoseStamped");

% Device_Arduino
arduino = serialport("/dev/ttyACM1",9600);
configureTerminator(arduino,"LF");
%% Sensors : GPS - IMU
global Pos; Pos = [0 0];
global theta;

x = 0.0;
y = 0.0;
theta_0 = 90.0;

% Waiting for connection to come through
tic
while (toc < 1)
end

% ロボットの初期位置推定
tic
while (toc < 1)
    theta = theta_0 + GetTheta(imu_sub);
    Pos = GetPosition(gps_sub, theta);
end
%% Set main goals 
length = 7;
width = 4;
n_width = 0.5;
[Points, n] = CreateMapPoints(length, width, n_width);

global P;P = [];
for i=1:n
    P_i = Pos + Points(i,:);
    P(i,:) = P_i;
end

%% Generate local goal

% ロボットの初期位置
global start_point; start_point=P(1,:);
% 終了ポイント
global main_goal; main_goal = P(2,:)';
% ロボットの初期状態 [x(m),y(m),yaw(Rad),v(m/s),steering_rate ω(rad/s)]
global robot_state; robot_state = [Pos(1) Pos(2) deg2rad(theta) 0 0]';
global goal;
global flag_goal;flag_goal = 0;
global flag_A;flag_A = 0;
global pdis;pdis = 1.0;
global distp2;distp2 = 1.0;
global plus; plus = 1.0;  %ゴール生成位置 自機＋〇

goal = GenerateGoal(start_point, plus, main_goal, robot_state);

%% RUN ROBOT
for i = 1:1
    for ii = 1:1:(n-1)
        % Initial flag
        flag = 0;           % phi angle flag
        flag_A = 0;         % last goal flag
        flag_goal = 0;      % local goal reach main goal flag

        % Calculate path for point to point
        [a,b,c] = CalculatePath(P(ii,:),P(ii+1,:));
        param(1,:) = [a b c];
        c_phi = @(x,y,theta)CalculatePhi(P(ii,:),P(ii+1,:),param(1,:),x,y,theta,distp2);

        start_point = P(ii,:);
        main_goal = P(ii+1,:)';
        goal = GenerateGoal(start_point, plus, main_goal, robot_state);

        Tstart = tic;
        tic

        while (flag == 0)

            while true
                % Get Current State
                theta = theta_0 + GetTheta(imu_sub);
                Pos = GetPosition(gps_sub, theta);
                x = Pos(1);
                y = Pos(2);

                % Set No detection at first and first run mess MODE
                detect = 0;
                velocity = 1;
                if (DWA == 5)
                    DWA = 0;
                    DWA_go = 0;
                    count = 0;
                end

                if (toc >= 1)
                    tic
                    scanMsg1 = receive(right_laser_sub,10);
                    scanMsg2 = receive(left_laser_sub,10);
                    n = 1;
                    % 検出工程2(Left)
                    while(n <= scan_max)
                        if(((15)/angle_increament < n) && (n <= (180-L_theta1)/angle_increament))%側方検出範囲
                            if(scanMsg2.Ranges(n) <= distance_short && scanMsg2.Ranges(n)~= Inf)
                                detect = 1;
                            end
                        elseif((0 < n)&&(n <= (15)/angle_increament) || ((270+L_theta2)/angle_increament <= n)&&(n < scan_max))%前方検出範囲
                            if(scanMsg2.Ranges(n) <= distance_long && scanMsg2.Ranges(n)~= Inf)
                                detect = 1;
                            end
                        end
                        n = n+1;
                    end

                    % Obstacle detection check LiDAR right
                    scanMsgRight = receive(right_laser_sub,10);
                    n = 1;

                    % 検出工程1(Right)
                    while(n <= scan_max)
                        if(((180+L_theta1)/angle_increament <= n) && (n < (345)/angle_increament))%側方検出範囲
                            if(scanMsg1.Ranges(n) <= distance_short && scanMsg1.Ranges(n)~= Inf)
                                detect = 1;
                            end
                        elseif((0 < n)&&(n <= (L_theta2)/angle_increament) || ((345)/angle_increament <= n)&&(n < scan_max))%前方検出範囲
                            if(scanMsg1.Ranges(n) <= distance_long && scanMsg1.Ranges(n)~= Inf)
                                detect = 1;
                            end
                        end
                        n = n+1;
                    end

                    %行動設定
                    if(detect == 1 && DWA == 0 && DWA_go == 0 && count == 0)%一時停止措置
                        stay=tic;
                        while(toc(stay) <= 1)%1秒待機
                            fprintf("Stop for Obstacle\n");
                            com_str = sprintf("%d,290",com_s);
                            writeline(arduino,com_str);
                            velocity = 0;
                            count = 1;
                        end
                    elseif(detect == 1 && count ==1)%DWAに切り替え
                        DWA_go = 1;
                    end
                    
                    [phi,flag] = c_phi(x,y,theta);

                    % AT DWA MODE
                    if(DWA_go == 1)
                        fprintf('Dynamic Window Approach start!!')
                        Pos = GetPosition(gps_sub, theta);
                        [u, mx, result, obstacle] = RunDWA(dt,scanMsg1,scanMsg2,goal,Pos,theta,result);
                        %アクセル踏込量設定
                        com_ac = round((u(1,1)+1.93)/0.003486);
                        if(com_ac > 680)
                            com_ac = 680;
                        elseif(com_ac < 580)%290
                            com_ac = 580;%290
                        end
                        %ステアリング量設定
                        %com_st = round(238.78+rad2deg(u(2,1))/0.2362);
                        %com_st = round((rad2deg(asin(0.95*u(2,1)/0.27))+70.86)/0.2362);
                        com_st = round((rad2deg(asin(0.95*u(2,1)/0.27))+127.26)/0.2362);
                        %操舵角設定
                        if(com_st > 760)
                            com_st = 760;
                        elseif (com_st < 370)
                            com_st = 370;
                        end
                        %指令値
                        com_str = sprintf("%d,%d",com_st,com_ac);%発進措置
                        writeline(arduino,com_str);
                        DWA = DWA + 1;
                    elseif(velocity == 1)%直進時
                        com_s = round(538.78+phi/0.2362);%538.78
                        if(com_s > 760)%760
                            com_s = 760;
                        elseif(com_s < 370)%370
                            com_s = 370;
                        end
                        com_a = 630;
                        if(com_a > 680)
                            com_a = 680;
                        elseif (com_a < 580)%290
                            com_a = 580;%290
                        end
                    com_str = sprintf("%d,%d",com_s,com_a);%発進措置
                    writeline(arduino,com_str);
                    end

                    if (readline(arduino) == "error")
                        fprintf("error\n");
                    end

                    if(flag_goal == 0)
                        goal = GenerateGoal(start_point, plus, main_goal, robot_state);
                    end

                    if (norm(main_goal(1:2)-goal(1:2))<pdis  && flag_goal == 0)
                        goal = main_goal;
                        flag_goal = 1;
                    end

                    %ゴール判定
                    if (norm(robot_state(1:2)-goal(1:2))<0.5  && flag_goal == 0)
                        goal = main_goal;
                        flag_goal = 1;
                    end

                    %メインゴール到達時, 終了の判定
                    if norm(robot_state(1:2)-main_goal(1:2))<pdis
                        disp('Arrive Goal!!');
                        flag_A = 1;
                    end

                    break
                end
            end
        end
        toc(Tstart)
    end
end

%% LOG

writeline(arduino,"stop");
readline(arduino)

rosshutdown %ROS stop

hold off
diary off

% fclose(id);
clear arduino

tic
while (toc < 1)
end

clear tcp
clear gps
clear arduino

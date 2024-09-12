%大学取材用（12/22）(1/17)

clear all
format compact
%% Refactored parameters 
dt = 1.0;   %刻み時間[s]

global result;result.x = [];
global Pos;
global theta;
global P1;
global P2;
global P3;
global P4;
global P;

global scanMsg1;
global scanMsg2;
global obstacle;
global goal;
global flag_goal;flag_goal = 0;
global flag_A;flag_A = 0;
global pdis;pdis = 1.5;
global distp2;distp2 = 1.5;
global plus; plus = 1.5;  %ゴール生成位置 自機＋〇

x = 0.0;
y = 0.0;
theta = 0.0;
theta_0 = 97.0;

rz_offset = 0.0;
offset_count = 0;

%%
n = 1;                  %点群データn番目
n_max = 720;            %総データ数
L_theta1 = 30;          %判定除外角 50 : arctan(39/33)
L_theta2 = 45;
distance_long = 1.7;    %閾値(長)
distance_short = 0.5;   %閾値(短)
%%
DWA = 0;
DWA_go = 0;
velocity = 1;
count = 0;
com_s = 537;

%% Device connection setup
% LIDAR SETUP
rosinit('127.0.0.1');
%ノード設定
node = ros.Node('rplidar_ros_to_matlab');
%LiDARデータのサブスクライバ
right_laser_sub = ros.Subscriber(node,"/left_scan","sensor_msgs/LaserScan");
left_laser_sub = ros.Subscriber(node,"/right_scan","sensor_msgs/LaserScan");

% Device_Arduino
arduino = serialport("COM4",9600);
configureTerminator(arduino,"LF");

% Device_IMU
tcp = tcpclient("localhost",10000);
configureTerminator(tcp,"LF","CR/LF");
flush(tcp);
writeline(tcp,"start");

% Get IMU Data 0 -> Initial Theta_0
tic
while (toc < 1)
end

flush(tcp)
tic
while(toc < 10)
    if(tcp.NumBytesAvailable >= 10)
        theta = ReadTheta(tcp,theta,rz_offset);
        offset_count = offset_count+1;
    end
end
rz_offset = theta/(offset_count/(1/0.15))*100;
theta = theta_0;

% Device_GPS
port = 'COM9';
[time,Pos] = ReadPosition2(port); % Get current Position of GPS
P1 = Pos;
P2 = P1+[0 4];
P3 = P1+[-3 4];
P4 = P1+[-3 0];

P = [P1; P2; P3; P4; P1];

%%

global start; start=[P1(1) P1(2)];                      %ロボットの初期位置
global main_goal; main_goal = [P2(1),P2(2)]';           %終了ポイント
global mx; mx = [Pos(1) Pos(2) deg2rad(theta) 0 0]';    %ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]

%
% goal = GenerateGoal(start, plus, main_goal, mx, goal);

%% RUN ROBOT
for i = 1:1
    for ii = 1:1:4
        
        % Initial flag
        flag = 0;
        flag_A = 0;
        flag_goal = 0;
        
        % Calculate path for point to point
        [a,b,c] = CalculatePath(P(ii,:),P(ii+1,:));
        param(1,:) = [a b c];
        c_phi = @(x,y,theta)CalculatePhi(P(ii,:),P(ii+1,:),param(1,:),x,y,theta,distp2);

        % SET GOAL
        if(ii == 1)
            start = [P1(1) P1(2)];
            main_goal = [P2(1) P2(2)]';
        end
        if(ii == 2)
            start = [P2(1) P2(2)];
            main_goal = [P3(1) P3(2)]';
        end
        if(ii == 3)
            start = [P3(1) P3(2)];
            main_goal = [P4(1) P4(2)]';
        end
        if(ii == 4)
            start = [P4(1) P4(2)];
            main_goal = [P1(1) P1(2)]';
            distp2 = 0.7;
        end
        
        goal = GenerateGoal(start, plus, main_goal, mx, goal);
        
        Tstart = tic;
        tics
        
        while (flag == 0)
            
            while true
                % Get Current State
                [time,Pos] = ReadPosition2(port);
                
                x = Pos(1);
                y = Pos(2);
               
                if(tcp.NumBytesAvailable >= 10)
                    theta = rad2deg(angdiff(0,deg2rad(ReadTheta(tcp,theta,rz_offset))));
                end
                
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
                    
                    % Obstacle detection check LiDAR left
                    scanMsg2 = receive(left_laser_sub,10);
                    n = 1;
                    
                    %検出工程2(Left)
                    while(n <= n_max)
                        if(((15)/0.5 < n) && (n <= (180-L_theta1)/0.5))%側方検出範囲
                            if(scanMsg2.Ranges(n) <= distance_short)
                                detect = 1;
                            end
                        elseif((0 < n)&&(n <= (15)/0.5) || ((270+L_theta2)/0.5 <= n)&&(n < n_max))%前方検出範囲
                            if(scanMsg2.Ranges(n) <= distance_long )
                                detect = 1;
                            end
                        end
                        n = n+1;
                    end
                    
                    
                    % Obstacle detection check LiDAR right
                    scanMsg1 = receive(right_laser_sub,10);
                    n = 1;
                    
                    %検出工程1(Right)
                    while(n <= n_max)
                        if(((180+L_theta1)/0.5 <= n) && (n < (345)/0.5))%側方検出範囲
                            if(scanMsg1.Ranges(n) <= distance_short)
                                detect = 1;
                            end
                        elseif((0 < n)&&(n <= (L_theta2)/0.5) || ((345)/0.5 <= n)&&(n < n_max))%前方検出範囲
                            if(scanMsg1.Ranges(n) <= distance_long )
                                detect = 1;
                            end
                        end
                        n = n+1;
                    end
                    
                    [phi,flag] = c_phi(x,y,theta);
                    
                    %行動設定
                    % AT DETECTION MODE
                    if(detect == 1 && DWA == 0 && DWA_go == 0 && count == 0)%一時停止措置
                        stay=tic;
                        while(toc(stay) <= 1)%1秒待機
                            com_str = sprintf("%d,290",com_s);
                            writeline(arduino,com_str);
                            velocity = 0;
                            count = 1;
                        end
                    elseif(detect == 1 && count ==1)%DWAに切り替え
                        DWA_go = 1;
                    end
                    
%                     [phi,flag] = c_phi(x,y,theta);
                    
                    % AT DWA MODE
                    if(DWA_go == 1)
                        disp('Dynamic Window Approach start!!')
%                         [time,Pos] = ReadPosition(id,x,y,theta);
                        [time,Pos] = ReadPosition2(port);

                        [u, mx, result, obstacle] = RunDWA(dt,scanMsg1,scanMsg2,goal,Pos,theta,result);
                        %アクセル踏込量設定
                        com_ac = round((u(1,1)+1.93)/0.003486);
                        if(com_ac > 680)
                            com_ac = 680;
                        elseif(com_ac < 580)%290
                            com_ac = 580;%290
                        end
                        %ステアリング量設定

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
                    %com_str = sprintf("%d,630",com);
                    writeline(arduino,com_str);
%                     com_str = sprintf("%d,630",com);
%                     writeline(arduino,com_str);
                    end
                    
                    
                    if (readline(arduino) == "error")
                        fprintf("error\n");
                    end
                    
                    if(flag_goal == 0)
                        goal = GenerateGoal(start, plus, main_goal, mx, goal);
%                         before_goal = goal;
%                         if(before_goal(1) > goal(1) || before_goal(2) > goal(2))
%                             goal = before_goal;
%                         end
                    end
                    
                    if (norm(main_goal(1:2)-goal(1:2))<pdis  && flag_goal == 0)
                        goal = main_goal;
                        flag_goal = 1;
                    end
                    
                    %ゴール判定
                    if (norm(mx(1:2)-goal(1:2))<0.5  && flag_goal == 0)
                        goal = main_goal;
                        flag_goal = 1;
                    end
                    
                    %メインゴール到達時, 終了の判定
                    if norm(mx(1:2)-main_goal(1:2))<pdis
                        disp('Arrive Goal!!');
                        flag_A = 1;
                    end
                    
                    fprintf("%f,%f,%f,%f,%f,%s\n",time,x,y,theta,phi,com_str);
                    quiver(x,y,cos(deg2rad(theta)),sin(deg2rad(theta)),0.5,'Color','b','Marker','o');
                    plot(main_goal(1),main_goal(2),'*m');hold on;
                    plot(goal(1),goal(2),'*r');hold on;
                    if(DWA > 0)
                        plot(obstacle(1),obstacle(2),'*k');hold on;
                        %plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
                    end
                    axis equal
                    drawnow
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
xlim([-5 10])
ylim([-20 -5])
saveas(figdata,"EXdata\result");
saveas(figdata,"EXdata\result",'png');

diary off

fclose(id);
clear arduino
writeline(tcp,"stop");

tic
while (toc < 1)
end

clear tcp
clear gps

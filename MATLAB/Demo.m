%大学取材用（12/22）(1/17)
clc
clear
close all
format compact
%% Refactored parameters 
dt = 1.0;   %刻み時間[s]
global result;result.x = [];

%%
DWA = 0;
DWA_go = 0;
velocity = 1;
count = 0;
com_s = 537;

%% Connection setup
[publisher, subscriber] = InitROS();

% Device_Arduino
arduino = serialport("/dev/ttyACM0",9600);
configureTerminator(arduino,"LF");
%% Sensors : GPS - IMU
theta = GetTheta(subscriber.imu);
Pos = GetPosition(subscriber.gps, theta);
% ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
global robot_state; robot_state = [Pos(1) Pos(2) deg2rad(theta) 0 0]';

%% Generate Trajectory
% Set Goals 
width = 3.8;
length = 7;

P = [];
P1 = Pos;
P2 = P1 + [length 0];
P3 = P1 + [length width];
P4 = P1 + [0 width];

% Make looping run
loop_time = 2;
for i = 1:loop_time
    P = [P; P1; P2; P3; P4];
end
% End trajectory with the first point
P = [P; P1];

%% Generate local goal
% ロボットの初期位置
global start_point; start_point= P(1,:);
global main_goal; main_goal = P(2,:)';
global goal;

% Set goal tolerance
flag_goal = 0;
flag_A = 0;
pdis = 1.5;
distp2 = 1.5;
plus = 1.5;

%% RUN ROBOT
for ii = 1:1:(height(P)-1)
    % Initial flag
    flag = 0;           % temp goal flag
    flag_A = 0;         % last goal flag
    flag_goal = 0;      % local goal reach main goal flag
    
    % Calculate path for point to point
    [a,b,c] = CalculatePath(P(ii,:),P(ii+1,:));
    c_phi = @(x,y,theta)CalculatePhi(P(ii,:),P(ii+1,:),[a b c],x,y,theta,distp2);

    % SET GOAL
    start_point = P(ii,:); 
    main_goal = P(ii+1,:)';
    if (main_goal == P(height(P)-1,:))
        distp2 = 0.7;
    end
    goal = GenerateGoal(start_point, plus, main_goal, robot_state);

    disp(main_goal);

    Tstart = tic;
    tic

    while (flag == 0)
        while true
            % Get Current State
            theta = GetTheta(subscriber.imu);
            Pos = GetPosition(subscriber.gps, theta);

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
                % Start Detecting
                [detect, scan_right] = LidarScan(subscriber.right_lidar, detect);
                [detect, scan_left] = LidarScan(subscriber.left_lidar, detect);
                detect = 0;
                
                % AT DETECTION MODE:
                % Stop 1 second if detected obstacle
                if(detect == 1 && DWA == 0 && DWA_go == 0 && count == 0)
                    stay=tic;
                    while(toc(stay) <= 1)
                        com_str = sprintf("%d,290",com_s);
                        writeline(arduino,com_str);
                        velocity = 0;
                        count = 1;
                    end
                % Change to DWA
                elseif (detect == 1 && count == 1)
                    DWA_go = 1;
                end

                % AT DWA MODE
                if(DWA_go == 1)

                    theta = GetTheta(subscriber.imu);
                    Pos = GetPosition(subscriber.gps, theta);

                    [u, robot_state, result, obstacle] = RunDWA(dt,scan_right,scan_left,goal,Pos,theta,result);
                    
                    % System Identifycation : 
                    com_ac = round((u(1,1)+1.93)/0.003486);
                    com_ac = min(max(com_ac, 580), 680);

                    com_st = round((rad2deg(asin(0.95*u(2,1)/0.27))+127.26)/0.2362);
                    com_st = min(max(com_st, 370), 760);

                    %指令値
                    com_str = sprintf("%d,%d",com_st,com_ac);%発進措置
                    writeline(arduino,com_str);
                    DWA = DWA + 1;

                elseif (DWA_go == 0 && velocity == 1)%直進時
                    [phi,flag] = c_phi(Pos(1),Pos(2),theta);

                    com_s = round(538.78+phi/0.2362);%538.78
                    com_s = min(max(com_s, 370), 760);
                    com_a = 630;

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

                % fprintf("x:%f,y:%f,theta:%f,phi:%f, str:%s \n",x,y,theta,phi,com_str);
                quiver(Pos(1),Pos(2),cos(deg2rad(theta)),sin(deg2rad(theta)),0.5,'Color','b','Marker','o');
                plot(main_goal(1),main_goal(2),'*m');hold on;
                plot(goal(1),goal(2),'*r');hold on;
                if(DWA > 0)
                    plot(obstacle(1),obstacle(2),'*k');hold on;
                end
                axis equal
                drawnow
                break
            end
        end
    end
    toc(Tstart)
end

%% LOG

writeline(arduino,"stop");
readline(arduino)
clear arduino
writeline(tcp,"stop");

tic
while (toc < 1)
end

clear tcp
clear gps
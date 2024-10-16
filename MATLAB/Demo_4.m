%大学取材用（09/18)
clc; clear; close all;
format compact;

%% Refactored parameters 
dt = 1.0;   %刻み時間[s]    
global result;result.x = [];

%% Connection setup
[publisher, subscriber] = InitROS();

%% Feedforward
com_s = 545;
v = 0.4;
com_a = round((v+1.93)/0.003486);

com_s_stop = 545;
com_a_stop = 290;
com_stop = sprintf("%d,%d",com_s_stop,com_a_stop);


% Device_Arduino
arduino = serialport("/dev/ttyACM0",9600);
configureTerminator(arduino,"LF");
sleep(2);

Tstart = tic;  % Start the overall timer
while toc(Tstart) < 10  % Feedforward for 10 seconds
    com_str = sprintf("%d,%d",com_s,com_a);
    writeline(arduino,com_str);
end

tic
while toc<3
    writeline(arduino,com_stop);
    theta0 = 0.0;
    theta = theta0 + GetTheta(subscriber.imu);
clear a    Pos0 = GetPosition(subscriber.gps, theta);
    % disp(Pos0);
end

%% Sensors : GPS - IMU

theta0 = 0.0;
theta = theta0 + GetTheta(subscriber.imu);
Pos = GetPosition(subscriber.gps, theta);

% Get robot initial position [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
robot_state = [Pos(1) Pos(2) deg2rad(theta) 0 0]';

%% Trajectory generation
Pos1 = Pos0 + [0 -1];
length = 4;
width = 7.5;
[P, n] = CreateMapPoints(length, width, Pos1);
P1 = P(1,:);
P2 = P(2,:);
P3 = P(3,:) + [0 1];
P4 = P(4,:) + [0 -0.5];
P5 = P(5,:) + [0 -1];
P6 = P(6,:) + [0 0.5];
P7 = P(7,:) + [0 1.0];
P8 = P(8,:) ;
P9 = P(9,:) ;
P10 = P(10,:) + [0 0.5];
P11 = P(11,:) + [0 0.5];
P12 = P(12,:) + [0 -1.0];

P = [P1;P2;P3;P4;P5;P6;P7;P8;P9;P10;P11;P12];
% Set goal tolerance
flag_goal = 0;
flag_A = 0;
pdis = 1.5;
distp2 = 1.5;
plus = 1.5;

%% RUN ROBOT
% Initial start points and first goal point.

for i = 1:1:(height(P)-1)
    % Initial flag
    flag = 0;           % temp goal flag
    flag_A = 0;         % last goal flag
    flag_goal = 0;      % temp goal reach main goal flag - after that : goal = main_goal
    
    % Motion planning
    % Calculate path for point to point
    [a,b,c] = CalculatePath(P(i,:),P(i+1,:));
    c_phi = @(x,y,theta)CalculatePhi(P(i,:),P(i+1,:),[a b c],x,y,theta,distp2);

    % Initial start points and goal point.
    start_point = P(i,:); 
    main_goal = P(i+1,:)';
    if (main_goal == P(height(P)-1,:))
        distp2 = 0.7;
    end
    goal = GenerateGoal(start_point, plus, main_goal, robot_state);
    Tstart = tic;
    tic

    while (flag == 0)
        % flag here present the robot_position compare to main_goal
        % If flag == 1 -> Robot_position to Main_goal position < than distp2

        while true
            % Get Current State
            theta = theta0 + GetTheta(subscriber.imu);
            Pos = GetPosition(subscriber.gps, theta);

            % Set No detection at first and first run mess MODE
            detect = 0;

            if (toc >= 1)
                tic
                % Start Detecting
                [detect, scan_right] = LidarScan(subscriber.right_lidar, detect);
                [detect, scan_left] = LidarScan(subscriber.left_lidar, detect);
                % AT DETECTION MODE:
                if (detect == 0)
                    [phi,flag] = c_phi(Pos(1),Pos(2),theta);

                    com_s = round(538.78+phi/0.2362);%538.78
                    com_s = min(max(com_s, 370), 760);
                    com_a = 630;

                    com_str = sprintf("%d,%d",com_s,com_a);
                    writeline(arduino,com_str);

                elseif (detect == 1)
                    stay = tic;
                    while(toc(stay) <= 1)
                        com_str = sprintf("%d,290",com_s);
                        writeline(arduino,com_str);
                    end
                end
               
                %% Generate goal
                % generate a new goal after send out control
                if(flag_goal == 0)
                    goal = GenerateGoal(start_point, plus, main_goal, robot_state);
                end

                % If above new generated goal is closed to main goal
                % then goal = main_goal
                if (norm(main_goal(1:2)-goal(1:2))<pdis  && flag_goal == 0)
                    goal = main_goal;
                    flag_goal = 1;
                end

                %ゴール判定
                % if new generated goal is too closed to robot state which
                % sometime happed near main goal then : goal = main goal
                if (norm(robot_state(1:2)-goal(1:2)) <0.5  && flag_goal == 0)
                    goal = main_goal;
                    flag_goal = 1;
                end

                % fprintf("x:%f,y:%f,theta:%f,phi:%f, str:%s \n",x,y,theta,phi,com_str);
                quiver(Pos(1),Pos(2),cos(deg2rad(theta)),sin(deg2rad(theta)),0.5,'Color','b','Marker','o');
                plot(main_goal(1),main_goal(2),'*m');hold on;
                plot(goal(1),goal(2),'*r');hold on;
                axis equal
                drawnow
                break
            end
        end
    end
    toc(Tstart)
end

writeline(arduino,"stop");
readline(arduino)
clear arduino
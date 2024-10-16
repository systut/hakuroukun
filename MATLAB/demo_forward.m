%大学取材用（09/18)
clc; clear; close all;
format compact;
% Initiate ROS connection
[publisher, subscriber] = InitROS();

% Device_Arduino
arduino = serialport("/dev/ttyACM0",9600);
configureTerminator(arduino,"LF");

theta0 = 0.0;
com_s = 545;
v = 0.4;
com_a = round((v+1.93)/0.003486);

sleep(2);

Tstart = tic;  % Start the overall timer
while toc(Tstart) < 10  % Loop runs until 10 seconds have passed
    %% Sensors : GPS - IMU
    theta = theta0 + GetTheta(subscriber.imu);
    Pos = GetPosition(subscriber.gps, theta);
    disp(Pos);
    com_str = sprintf("%d,%d",com_s,com_a);
    writeline(arduino,com_str);
end

writeline(arduino,"stop");
readline(arduino)
clear arduino; 
close all;
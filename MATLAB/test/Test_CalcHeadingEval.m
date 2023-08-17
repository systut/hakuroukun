global Pos;
global theta;
global mx;
global goal;
global start;
global plus;
global main_goal;
global P1;
global P2;
% x = 0.0;
% y = 0.0;
theta = 0.0;
theta_0 = 97.0;
% rz_offset = 0.0;
% offset_count = 0;
% 
% % Device GPS
% port = 'COM9';
% gps = gpsdev(port,'OutputFormat',"timetable");
% 
% % Device_IMU
% tcp = tcpclient("localhost",10000);
% configureTerminator(tcp,"LF","CR/LF");
% flush(tcp);
% writeline(tcp,"start");
% 
% flush(tcp)
% tic
% while(toc < 10)
%     if(tcp.NumBytesAvailable >= 10)
%         theta = ReadTheta(tcp,theta,rz_offset);
%         offset_count = offset_count+1;
%     end
% end
% rz_offset = theta/(offset_count/(1/0.15))*100;
% theta = theta_0;
% 
%     
% if(tcp.NumBytesAvailable >= 10)
%     theta = rad2deg(angdiff(0,deg2rad(ReadTheta(tcp,theta,rz_offset))));
% end
% 
% [Pos, time] = ReadPosition2(port);
theta = theta_0;
Pos = [0 0];
P1 = Pos;
P2 = Pos + [0 4];

fprintf("Current Pos : %f,%f,%f \n", Pos(1), Pos(2), theta);

start = [P1(1) P1(2)];
main_goal = [P2(1) P2(2)];
mx = [Pos(1) Pos(2) deg2rad(theta) 0 0]';
goal = GenerateGoal(start, plus, main_goal, mx, goal);


heading=CalcHeadingEval(mx,goal);
fprintf("Heading : %f \n",heading);

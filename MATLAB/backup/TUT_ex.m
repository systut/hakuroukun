%大学取材用（12/22）(1/17)
%外周一周のみ
%(2/6)kp変えてみる？ DWA切り替えタイミング変える？ステップ⇒LiDAR検出のみ
%横検出範囲広すぎると過敏に反応　要調整

clear all
format compact

%IPアドレス設定:mobile_wifi
rosinit("http://192.168.43.67:11311","NodeHost","192.168.43.108");
%ノード設定
node = ros.Node('test000');
%LiDARデータのサブスクライバ
laserSub1 = ros.Subscriber(node,"/front_scan","sensor_msgs/LaserScan");
laserSub2 = ros.Subscriber(node,"/back_scan","sensor_msgs/LaserScan");

global result;result.x = [];
global Pos;
global theta;
global P1;
global P2;
global P3;
global P4;
global P;
% global p3;
% global p4;
% global p5;
% global p6;
% global p7;
% global p8;
% global p9;
% global p10;
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

n = 1;          %点群データn番目
n_max = 720;   %総データ数
L_theta1 = 30;   %判定除外角 50 : arctan(39/33)
L_theta2 = 45;
distance_long = 1.7; %閾値(長)
distance_short = 0.5; %閾値(短)
%alpha = 90 - acos(distance_short/distance_long)*180/pi; %前方側部境界角
DWA = 0;
DWA_go = 0;
velocity = 1;
count = 0;
com_s = 550;%550

delete EXdata\DWAlog.txt
diary EXdata\DWAlog.txt

figdata = figure;
hold on
box on

arduino = serialport("COM4",9600);
configureTerminator(arduino,"LF");

tcp = tcpclient("localhost",10000);
configureTerminator(tcp,"LF","CR/LF");
flush(tcp);
writeline(tcp,"start");

tic
while (toc < 1)
end

flush(tcp)
tic
while(toc < 10)
    if(tcp.NumBytesAvailable >= 10)
        theta = read_theta(tcp,theta,rz_offset);
        offset_count = offset_count+1;
    end
end
rz_offset = theta/(offset_count/(1/0.15))*100
theta = theta_0;

id = fopen('EXdata\gps','r');
[time,Pos] = read_pos(id,x,y,theta)

P1 = Pos;
P2 = P1+[-0.73 5.95];%-0.73 5.95
P3 = P1+[-3.91 5.55];%-3.91 5.55
P4 = P1+[-3.24 -0.68];%-3.24 -0.68
% P2 = P1+[-0.61 4.96];%-0.73 5.95
% P3 = P1+[-3.59 4.6];%-3.91 5.55
% P4 = P1+[-2.95 -0.61];%-3.24 -0.68
% E14 = (P4-P1)/5;
% E23 = (P3-P2)/5;
P = [P1; P2; P3; P4; P1];

% p3 = P2+3*E23;
% p4 = P1+3*E14;
% p5 = P1+E14;
% p6 = P2+E23;
% p7 = P2+4*E23;
% p8 = P1+4*E14;
% p9 = P1+2*E14;
% p10 = P2+2*E23;

global start; start=[P1(1) P1(2)];%ロボットの初期位置
global main_goal; main_goal = [P2(1),P2(2)]'; %終了ポイント
global mx; mx = [Pos(1) Pos(2) deg2rad(theta) 0 0]';%ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]

goal_generate

for i = 1:1
    for ii = 1:1:4
        flag = 0;
        flag_A = 0;
        flag_goal = 0;
        
        [a,b,c] = calc_path(P(ii,:),P(ii+1,:));
        param(1,:) = [a b c];
        
        c_phi = @(x,y,theta)calc_phi(P(ii,:),P(ii+1,:),param(1,:),x,y,theta);
        
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
        
        goal_generate
        
        Tstart = tic;
        tic
        while (flag == 0)%flag == 0
            %flush(tcp);
            %tcp.NumBytesAvailable
            
            while true
                if(tcp.NumBytesAvailable >= 10)
                    theta = rad2deg(angdiff(0,deg2rad(read_theta(tcp,theta,rz_offset))));
                end
                
                [time,Pos] = read_pos(id,x,y,theta);
                x = Pos(1);
                y = Pos(2);
                
                detect = 0;
                velocity = 1;
                if (DWA == 5)
                    DWA = 0;
                    DWA_go = 0;
                    count = 0;
                end
                
                if (toc >= 1)
                    tic
                    
                    scanMsg2 = receive(laserSub2,10);
                    n = 1;
                    
                    %検出工程2(Left)
                    while(n <= n_max)
                        if(((15)/0.5 < n) && (n <= (180-L_theta1)/0.5))%側方検出範囲
                            if(scanMsg2.Ranges(n) <= distance_short)
                                detect = 1;
                                %DWA_go = 1;
                            end
                        elseif((0 < n)&&(n <= (15)/0.5) || ((270+L_theta2)/0.5 <= n)&&(n < n_max))%前方検出範囲
                            if(scanMsg2.Ranges(n) <= distance_long )
                                detect = 1;
                                %DWA_go = 1;
                            end
                        end
                        n = n+1;
                    end
                           
                    scanMsg1 = receive(laserSub1,10);
                    n = 1;
                    
                    %検出工程1(Right)
                    while(n <= n_max)
                        if(((180+L_theta1)/0.5 <= n) && (n < (345)/0.5))%側方検出範囲
                            if(scanMsg1.Ranges(n) <= distance_short)
                                detect = 1;
                                %DWA_go = 1;
                            end
                        elseif((0 < n)&&(n <= (L_theta2)/0.5) || ((345)/0.5 <= n)&&(n < n_max))%前方検出範囲
                            if(scanMsg1.Ranges(n) <= distance_long )
                                detect = 1;
                                %DWA_go = 1;
                            end
                        end
                        n = n+1;
                    end
                    
                    %行動設定
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
                    
                    [phi,flag] = c_phi(x,y,theta);
                    
                    if(DWA_go == 1)
                        disp('Dynamic Window Approach start!!')
                        [time,Pos] = read_pos(id,x,y,theta);
                        u = DWA_1();
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
                        if(com_st > 760)%440
                            com_st = 760;
                        elseif (com_st < 370)%370 50
                            com_st = 370;
                        end
                        %指令値
                        com_str = sprintf("%d,%d",com_st,com_ac);%発進措置
                        writeline(arduino,com_str);
                        DWA = DWA + 1;
                        %elseif(velocity == 0 && DWA == 0)%停止時
                        %com_str = sprintf("%d,290",com);
%                     else%直進時
%                         com = round(538.78+phi/0.2362);
%                     if(com > 760)
%                         com = 760;
%                     elseif (com < 370)
%                         com = 370;
%                     end
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
                        goal_generate
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


function u = DWA_1()
global Pos;
global theta;
global dt; dt=1.0;%刻み時間[s]
global result;
global goal;
%global flag_goal;
%global flag_A;
global obstacle;
global Kinematic;
global obstacleR;
%global main_goal;
global mx; mx = [Pos(1) Pos(2) deg2rad(theta) 0 0]';%ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]

[obx1,oby1,R1] = obodom1(mx);%障害物座標の計算1
[obx2,oby2,R2] = obodom2(mx);%障害物座標の計算2
[obx,oby] = obodom(obx1,oby1,obx2,oby2,R1,R2);%最近障害物座標の判別
obstacle = [obx oby];%障害物座標
obstacleR = 0.3;%衝突判定用の障害物の半径

%ロボットの力学モデル
%[最高速度[m/s],最高回頭速度[rad/s],最高加減速度[m/ss],最高加減回頭速度[rad/ss],
% 速度解像度[m/s],回頭速度解像度[rad/s]]
Kinematic=[0.27,deg2rad(10.0),0.27,deg2rad(10.0),0.03,deg2rad(1.0)];
%Kinematic=[0.27,deg2rad(10.0),0.27,deg2rad(10.0),0.27,deg2rad(1.0)];

%評価関数のパラメータ [heading,dist,velocity,predictDT]
evalParam=[1.0,9.8,10.0,2.0];
% area=[-1 11 -1 11];%シミュレーションエリアの広さ [xmin xmax ymin ymax]

result.x= [];
%flag_goal = 0;

%DWAによる入力値の計算
[u,traj]=DynamicWindowApproach(mx,Kinematic,goal,evalParam,obstacle,obstacleR);
mx=f(mx,u);%運動モデルによる移動

%シミュレーション結果の保存
result.x=[result.x; mx'];

end

function [u,trajDB]=DynamicWindowApproach(mx,model,goal,evalParam,obstacle,obstacleR)
%DWAによる入力値の計算をする関数

%Dynamic Window[vmin,vmax,ωmin,ωmax]の作成
Vr=CalcDynamicWindow(mx,model);
%評価関数の計算
[evalDB,trajDB]=Evaluation(mx,Vr,goal,obstacle,obstacleR,model,evalParam);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

%各評価関数の正規化
evalDB=NormalizeEval(evalDB);

%最終評価値の計算
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

[maxv,ind]=max(feval);%最も評価値が大きい入力値のインデックスを計算
u=evalDB(ind,1:2)';%評価値が高い入力値を返す
end

function [evalDB,trajDB]=Evaluation(mx,Vr,goal,obstacle,obstacleR,model,evalParam)
%各パスに対して評価値を計算する関数
evalDB=[];
trajDB=[];

for vt=Vr(1):model(5):Vr(2)
    for ot=Vr(3):model(6):Vr(4)
        %軌跡の推定
        [xt,traj]=GenerateTrajectory(mx,vt,ot,evalParam(4),model);
        %各評価関数の計算
        heading=CalcHeadingEval(xt,goal);
        dist=CalcDistEval(xt,obstacle,obstacleR);
        vel=abs(vt);

        evalDB=[evalDB;[vt ot heading dist vel]];
        trajDB=[trajDB;traj];
    end
end
end

function EvalDB=NormalizeEval(EvalDB)
%評価値を正規化する関数
if sum(EvalDB(:,3))~=0
    EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
end
if sum(EvalDB(:,4))~=0
    EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~=0
    EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
end
end

function [mx,traj]=GenerateTrajectory(mx,vt,ot,evaldt,model)
%軌跡データを作成する関数
global dt;
time=0;
u=[vt;ot];%入力値
traj=mx;%軌跡データ
while time<=evaldt
    time=time+dt;%シミュレーション時間の更新
    mx=f(mx,u);%運動モデルによる推移
    traj=[traj mx];
end
end

function stopDist=CalcBreakingDist(vel,model)
%現在の速度から力学モデルに従って制動距離を計算する関数
global dt;
stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;%制動距離の計算
    vel=vel-model(3)*dt;%最高原則
end
end

function dist=CalcDistEval(mx,obstacle,obstacleR)
%障害物との距離評価値を計算する関数

dist=1.5;
for io=1:length(obstacle(:,1))
    disttmp=norm(obstacle(io,:)-mx(1:2)')-obstacleR;%パスの位置と障害物とのノルム誤差を計算
    if dist>disttmp%最小値を見つける
        dist=disttmp;
    end
end
end

function heading=CalcHeadingEval(mx,goal)
%headingの評価関数を計算する関数

R_theta=rad2deg(mx(3));%ロボットの方位
goalTheta=rad2deg(atan2(goal(2)-mx(2),goal(1)-mx(1)));%ゴールの方位

if goalTheta>R_theta
    targetTheta=goalTheta-R_theta;%ゴールまでの方位差分[deg]
else
    targetTheta=R_theta-goalTheta;%ゴールまでの方位差分[deg]
end

heading=180-targetTheta;
end

function Vr=CalcDynamicWindow(mx,model)
%モデルと現在の状態からDyamicWindowを計算
global dt;
%車両モデルによるWindow
Vs=[0 model(1) -model(2) model(2)];

%運動モデルによるWindow
Vd=[mx(4)-model(3)*dt mx(4)+model(3)*dt mx(5)-model(4)*dt mx(5)+model(4)*dt];

%最終的なDynamic Windowの計算
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%[vmin,vmax,ωmin,ωmax]
end

function mx = f(mx,u)
% Motion Model
global dt;

F = [1 0 0 0 0
    0 1 0 0 0
    0 0 1 0 0
    0 0 0 0 0
    0 0 0 0 0];

B = [dt*cos(mx(3)) 0
    dt*sin(mx(3)) 0
    0 dt
    1 0
    0 1];

mx= F*mx+B*u;
if(mx(3) >= 3*pi/2)
    mx(3) = mx(3)-2*pi;
end
end

function goal_generate
global start;
global plus;
global main_goal;
global mx;

%a = (main_goal(2)-start(2))/(main_goal(1)-start(1));
a1 = main_goal(1)-start(1);
a2 = main_goal(2)-start(2);
a = a2/a1;
b = -a*start(1)+start(2);
xg = (a*(mx(2)-b)+mx(1))/(a^2+1);
yg = a*xg+b;

theta_g = atan(a);
% A = plus*cos(theta_g);
% B = plus*sin(theta_g);
if(a1 < 0 && a2 < 0)
A = -plus*cos(theta_g);
B = -plus*sin(theta_g);
elseif(a1 > 0 && a2 < 0)
A = plus*cos(theta_g);
B = plus*sin(theta_g);
elseif(a1 < 0 && a2 > 0)
A = -plus*cos(theta_g);
B = -plus*sin(theta_g);
else%x,y:+
A = plus*cos(theta_g);
B = plus*sin(theta_g);
end
x2 = xg+A;
y2 = yg+B;

global goal; goal = [x2,y2]';%DWAゴールの位置 [x(m),y(m)] DWA時に逐次更新
if(main_goal(1) == start(1))%x座標が同じとき
    if(a2 < 0)
    goal = [start(1),mx(2)-plus]';
    else
    goal = [start(1),mx(2)+plus]';
    end
elseif(main_goal(2) == start(2))
    if(a1 < 0)
    goal = [mx(1)-plus,start(2)]';
    else
    goal = [mx(1)+plus,start(2)]';
    end
end
end

function [obx1,oby1,R1] = obodom1(mx)
%LiDARからみた障害物座標の計算1
global scanMsg1;

R1 = [scanMsg1.Ranges(1:90);  scanMsg1.Ranges(420:720)];
[val,idx] = min(R1);
m1 = idx;
if(m1 > 90)
    m1 = m1 + 329;
end

L_phi1 = CalcLphi1(m1);

R1 = R1(idx);

obx1 = R1*cos(mx(3)+deg2rad(L_phi1))+(mx(1)+0.33*cos(mx(3)+deg2rad(-70)));
oby1 = R1*sin(mx(3)+deg2rad(L_phi1))+(mx(2)+0.33*sin(mx(3)+deg2rad(-70)));
end

function [obx2,oby2,R2] = obodom2(mx)
%LiDARからみた障害物座標の計算2
global scanMsg2;

R2 = [scanMsg2.Ranges(1:300);  scanMsg2.Ranges(630:720)];
[val,idx] = min(R2);
m2 = idx;
if(m2 > 300)
    m2 = m2 + 329;
end

L_phi2 = CalcLphi2(m2);

R2 = R2(idx);

obx2 = R2*cos(mx(3)+deg2rad(L_phi2))+(mx(1)+0.33*cos(mx(3)+deg2rad(70)));
oby2 = R2*sin(mx(3)+deg2rad(L_phi2))+(mx(2)+0.33*sin(mx(3)+deg2rad(70)));
end

function L_phi1 = CalcLphi1(m1)
%LiDARの角度指定1
if (m1 <= 90)
    L_phi1 = m1*0.5;
elseif (m1 >= 420)
    L_phi1 = (m1*0.5) - 360;
end
end

function L_phi2 = CalcLphi2(m2)
%LiDARの角度指定2
if (m2 <= 300)
    L_phi2 = m2*0.5;
elseif (m2 >= 630)
    L_phi2 = (m2*0.5) - 360;
end
end

function [obx,oby] = obodom(obx1,oby1,obx2,oby2,R1,R2)
%最近座標判別
if(R1 < R2)
obx = obx1;
oby = oby1;
else
obx = obx2;
oby = oby2;
end
end

function ret = read_theta(tcp,theta,rz_offset)
str_imu = readline(tcp);
str_imu = erase(str_imu,char(0));

if(startsWith(str_imu,"ags,"))
    str_imu = erase(str_imu,"ags,");
    data_imu = str2double(strsplit(str_imu,','));
    rot_z = (data_imu(7)-rz_offset)/100*0.15;
    theta = theta+rot_z;
else
    fprintf("message：%s\n",str_imu);
end
ret = theta;
end


function [time,Pos] = read_pos(id,x,y,theta)
l1 = 0.4;
fseek(id,-500,1);
while not(feof(id))
    ch_gps = fgetl(id);
end
str_gps = string(ch_gps);

if(strlength(str_gps) == 144)
    Time = str2double(strsplit(strtrim(extractBetween(str_gps,12,23)),':'));
    time = Time(1)*3600+Time(2)*60+Time(3);
    str_gps = strtrim(extractBetween(str_gps,24,53));
    Pos = str2double(strsplit(str_gps));
    Pos = Pos+[l1*cos(deg2rad(theta)) l1*sin(deg2rad(theta))];
else
    fprintf("error：%s\n",str_gps);
    time = 0;
    Pos = [x y];
end
end


function [a,b,c] = calc_path(P1,P2)
A = [P1(1) 1; P2(1) 1];
B = [P1(2); P2(2)];

if(P1(1) == P2(1))
    a = 1;
    b = 0;
    c = -P1(1);
    line_y = linspace(P1(2),P2(2),100);
    line_x = -c;
else
    X = A\B;
    a = X(1);
    b = -1;
    c = X(2);
    line_x = linspace(P1(1),P2(1),100);
    line_y = a*line_x+c;
end
plot(line_x,line_y,'Color','r');
end


function [phi,flag] = calc_phi(P1,P2,param,x,y,theta)
global distp2;
a = param(1);
b = param(2);
c = param(3);
kp = 1.0;%変える？ 0.67 1.5
n = [a b];
Pn = [x y];

d = (a*Pn(1)+b*Pn(2)+c)/norm(n);
dis = abs(d);
Pr = Pn-d.*normr(n);

%P12<P1rでbreak
P12 = P2-P1;
P1r = Pr-P1;

theta_r = rad2deg(atan2(P12(2),P12(1)));
P1c = Pn-P1;
sgn = sign(cross([P1c 0],[P12 0]));
beta = rad2deg(atan(kp*sgn(3)*dis));
phi = rad2deg(angdiff(deg2rad(theta-theta_r),deg2rad(beta)));

% if(norm(P12) <= norm(P1r))%ここ変える？
%     flag = 1;
% else
%     flag = 0;
% end

distP = norm(P12) - norm(P1r);
if(distP <= distp2)
    flag = 1;
else
    flag = 0;
end
end
%追従・DWA併用,回避方法改善案1(stopを続ける),位置で速度変化
%速度切り替え地点を事前に指定(全周分)
%オーバーシュート対策　実装(12/5)
%(2/6)kp変えてみる？ DWA切り替えタイミング変える？ステップ⇒LiDAR検出のみ

clear all
format compact

%IPアドレス設定:mobile_wifi
rosinit("http://192.168.43.67:11311","NodeHost","192.168.43.108");
%rosinit("http://192.168.100.21:11311","NodeHost","192.168.43.108");
%ノード設定
node = ros.Node('test000');
%LiDARデータのサブスクライバ
laserSub1 = ros.Subscriber(node,"/front_scan","sensor_msgs/LaserScan");
laserSub2 = ros.Subscriber(node,"/back_scan","sensor_msgs/LaserScan");

global result;result.x = [];
global Pos;
global theta;
global P1;
global P1h;
global P2;
global P3;
global P3h;
global P4;
global P;
global p3;
global p3h;
global p4;
global p5;
global p5h;
global p6;
global p7;
global p7h;
global p8;
global p9;
global p9h;
global p10;
global p11;
global p11h;
global p12;
global p13;
global p13h;
global p14;
global p15;
global p15h;
global p16;
global p17;
global p17h;
global p18;
global p19;
global p19h;
global p20;
global p21;
global p21h;
global p22;
global p23;
global p23h;
global p24;
global p25;
global p25h;
global p26;
global p27;
global p27h;
global p28;
global p29;
global p29h;
global p30;
global scanMsg1;
global scanMsg2;
global obstacle;
global goal;
global flag_goal;flag_goal = 0;
global flag_A;flag_A = 0;
global stop;stop = 0;
global route;route = 1;

x = 0.0;
y = 0.0;
theta = 0.0;
theta_0 = 109.0;
rz_offset = 0.0;
offset_count = 0;

n = 1;          %点群データn番目
n_max = 720;   %総データ数
L_theta1 = 30;   %判定除外角 50 : arctan(39/33)
L_theta2 = 45;
distance_long = 2.0; %閾値(長)
distance_short = 0.7; %閾値(短)
%alpha = 90 - acos(distance_short/distance_long)*180/pi; %前方側部境界角
DWA = 0;
DWA_go = 0;
velocity = 1;
count = 0;
com_s = 550;

delete EXdata\log.txt
diary EXdata\log.txt

figdata = figure;
hold on
box on

arduino = serialport("COM4",9600);%COM5
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

P1 = Pos;%[27.0 -30.0]
P1h = [21.5 -15.0];
P2 = [16.5 -0.08];%[15.50 4.30]%half22.6 -14.8
P3 = [3.4 -3.5];%[1.50 -0.50]%half7.0 -19.6
P3h = [8.5 -19.0];
P4 = [13.50 -34.75];%[12.50 -34.50]

E14 = (P4-P1)/15;
E23 = (P3-P2)/15;
E13h = (P3h-P1h)/15;
p3 = P2+8*E23;
p3h = P1h+8*E13h;
p4 = P1+8*E14;
p5 = P1+E14;
p5h = P1h+E13h;
p6 = P2+E23;
p7 = P2+9*E23;
p7h = P1h+9*E13h;
p8 = P1+9*E14;
p9 = P1+2*E14;
p9h =P1h+2*E13h;
p10 = P2+2*E23;
p11 = P2+10*E23;
p11h = P1h+10*E13h;
p12 = P1+10*E14;
p13 = P1+3*E14;
p13h = P1h+3*E13h;
p14 = P2+3*E23;
p15 = P2+11*E23;
p15h = P1h+11*E13h;
p16 = P1+11*E14;
p17 = P1+4*E14;
p17h = P1h+4*E13h;
p18 = P2+4*E23;
p19 = P2+12*E23;
p19h = P1h+12*E13h;
p20 = P1+12*E14;
p21 = P1+5*E14;
p21h = P1h+5*E13h;
p22 = P2+5*E23;
p23 = P2+13*E23;
p23h = P1h+13*E13h;
p24 = P1+13*E14;
p25 = P1+6*E14;
p25h = P1h+6*E13h;
p26 = P2+6*E23;
p27 = P2+14*E23;
p27h = P1h+14*E13h;
p28 = P1+14*E14;
p29 = P1+7*E14;
p29h = P1h+7*E13h;
p30 = P2+7*E23;

P = [P1; P1h; P2; p3; p3h
    p4; p5; p5h; p6; p7
    p7h; p8; p9; p9h; p10
    p11; p11h; p12; p13; p13h
    p14; p15; p15h; p16; p17
    p17h; p18; p19; p19h; p20
    p21; p21h; p22; p23; p23h
    p24; p25; p25h; p26; p27
    p27h; p28; p29; p29h; p30
    P3; P3h; P4; P1];

global start; start=[P1(1) P1(2)];%ロボットの初期位置
global main_goal; main_goal = [P1h(1),P1h(2)]'; %終了ポイント
global mx; mx = [Pos(1) Pos(2) deg2rad(theta) 0 0]';%ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
global pdis;pdis = 1.5;%2.0
global plus; plus = 1.5;  %ゴール生成位置 自機＋〇 2.0
obstacle=[-100,-100];
      
%obstacle_generate

goal_generate

for i = 1:1
    for ii = 1:1:48
        flag = 0;
        flag_A = 0;
        flag_goal = 0;
        
        [a,b,c] = calc_path(P(ii,:),P(ii+1,:));
        param(1,:) = [a b c];
        
        c_phi = @(x,y,theta)calc_phi(P(ii,:),P(ii+1,:),param(1,:),x,y,theta);
        
        if(ii == 1)
            start = [P1(1) P1(2)];
            main_goal = [P1h(1) P1h(2)]';
            stop = 1;
        end
        if(ii == 2)
            start = [P1h(1) P1h(2)];
            main_goal = [P2(1) P2(2)]';
            stop = 1;
            route = 2
        end
        if(ii == 3)
            start = [P2(1) P2(2)];
            main_goal = [p3(1) p3(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 4)
            start = [p3(1) p3(2)];
            main_goal = [p3h(1) p3h(2)]';
            stop = 0;
            route = 3
        end
        if(ii == 5)
            start = [p3h(1) p3h(2)];
            main_goal = [p4(1) p4(2)]';
            stop = 0;
            route = 4
        end
        if(ii == 6)
            start = [p4(1) p4(2)];
            main_goal = [p5(1) p5(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 7)
            start = [p5(1) p5(2)];
            main_goal = [p5h(1) p5h(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 8)
            start = [p5h(1) p5h(2)];
            main_goal = [p6(1) p6(2)]';
            stop = 1;
            route = 2
        end
        if(ii == 9)
            start = [p6(1) p6(2)];
            main_goal = [p7(1) p7(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 10)
            start = [p7(1) p7(2)];
            main_goal = [p7h(1) p7h(2)]';
            stop = 0;
            route = 3
        end
        if(ii == 11)
            start = [p7h(1) p7h(2)];
            main_goal = [p8(1) p8(2)]';
            stop = 0;
            route = 4
        end
        if(ii == 12)
            start = [p8(1) p8(2)];
            main_goal = [p9(1) p9(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 13)
            start = [p9(1) p9(2)];
            main_goal = [p9h(1) p9h(2)]';
            stop = 0;
            route = 1
        end
        if(ii == 14)
            start = [p9h(1) p9h(2)];
            main_goal = [p10(1) p10(2)]';
            stop = 0;
            route = 2
        end
        if(ii == 15)
            start = [p10(1) p10(2)];
            main_goal = [p11(1) p11(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 16)
            start = [p11(1) p11(2)];
            main_goal = [p11h(1) p11h(2)]';
            stop = 0;
            route = 3
        end
        if(ii == 17)
            start = [p11h(1) p11h(2)];
            main_goal = [p12(1) p12(2)]';
            stop = 0;
            route = 4
        end
        if(ii == 18)
            start = [p12(1) p12(2)];
            main_goal = [p13(1) p13(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 19)
            start = [p13(1) p13(2)];
            main_goal = [p13h(1) p13h(2)]';
            stop = 0;
            route = 1
        end
        if(ii == 20)
            start = [p13h(1) p13h(2)];
            main_goal = [p14(1) p14(2)]';
            stop = 0;
            route = 2
        end
        if(ii == 21)
            start = [p14(1) p14(2)];
            main_goal = [p15(1) p15(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 22)
            start = [p15(1) p15(2)];
            main_goal = [p15h(1) p15h(2)]';
            stop = 0;
            route = 3
        end
        if(ii == 23)
            start = [p15h(1) p15h(2)];
            main_goal = [p16(1) p16(2)]';
            stop = 0;
            route = 4
        end
        if(ii == 24)
            start = [p16(1) p16(2)];
            main_goal = [p17(1) p17(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 25)
            start = [p17(1) p17(2)];
            main_goal = [p17h(1) p17h(2)]';
            stop = 0;
            route = 1
        end
        if(ii == 26)
            start = [p17h(1) p17h(2)];
            main_goal = [p18(1) p18(2)]';
            stop = 0;
            route = 2
        end
        if(ii == 27)
            start = [p18(1) p18(2)];
            main_goal = [p19(1) p19(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 28)
            start = [p19(1) p19(2)];
            main_goal = [p19h(1) p19h(2)]';
            stop = 0;
            route = 3
        end
        if(ii == 29)
            start = [p19h(1) p19h(2)];
            main_goal = [p20(1) p20(2)]';
            stop = 0;
            route = 4
        end
        if(ii == 30)
            start = [p20(1) p20(2)];
            main_goal = [p21(1) p21(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 31)
            start = [p21(1) p21(2)];
            main_goal = [p21h(1) p21h(2)]';
            stop = 0;
            route = 1
        end
        if(ii == 32)
            start = [p21h(1) p21h(2)];
            main_goal = [p22(1) p22(2)]';
            stop = 0;
            route = 2
        end
        if(ii == 33)
            start = [p22(1) p22(2)];
            main_goal = [p23(1) p23(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 34)
            start = [p23(1) p23(2)];
            main_goal = [p23h(1) p23h(2)]';
            stop = 0;
            route = 3
        end
        if(ii == 35)
            start = [p23h(1) p23h(2)];
            main_goal = [p24(1) p24(2)]';
            stop = 0;
            route = 4
        end
        if(ii == 36)
            start = [p24(1) p24(2)];
            main_goal = [p25(1) p25(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 37)
            start = [p25(1) p25(2)];
            main_goal = [p25h(1) p25h(2)]';
            stop = 0;
            route = 1
        end
        if(ii == 38)
            start = [p25h(1) p25h(2)];
            main_goal = [p26(1) p26(2)]';
            stop = 0;
            route = 2
        end
        if(ii == 39)
            start = [p26(1) p26(2)];
            main_goal = [p27(1) p27(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 40)
            start = [p27(1) p27(2)];
            main_goal = [p27h(1) p27h(2)]';
            stop = 1;
            route = 3
        end
        if(ii == 41)
            start = [p27h(1) p27h(2)];
            main_goal = [p28(1) p28(2)]';
            stop = 1;
            route = 4
        end
        if(ii == 42)
            start = [p28(1) p28(2)];
            main_goal = [p29(1) p29(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 43)
            start = [p29(1) p29(2)];
            main_goal = [p29h(1) p29h(2)]';
            stop = 0;
            route = 1
        end
        if(ii == 44)
            start = [p29h(1) p29h(2)];
            main_goal = [p30(1) p30(2)]';
            stop = 0;
            route = 2
        end
        if(ii == 45)
            start = [p30(1) p30(2)];
            main_goal = [P3(1) P3(2)]';
            stop = 1;
            route = 1
        end
        if(ii == 46)
            start = [P3(1) P3(2)];
            main_goal = [P3h(1) P3h(2)]';
            stop = 1;
            route = 3
        end
        if(ii == 47)
            start = [P3h(1) P3h(2)];
            main_goal = [P4(1) P4(2)]';
            stop = 1;
            route = 4
        end
        if(ii == 48)
            start = [P4(1) P4(2)];
            main_goal = [P1(1) P1(2)]';
            stop = 1;
            route = 1
        end
        
        goal_generate
        
        Tstart = tic;
        tic
        while(flag == 0)%flag_A == 0
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
                %stop = 0;
                velocity = 1;
                if (DWA == 10)%5,10
                    count = 0;
                    DWA = 0;
                    DWA_go = 0;
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
                    
%                     [obx1,oby1,R1] = obodom1(mx);%障害物座標の計算1
%                     [obx2,oby2,R2] = obodom2(mx);%障害物座標の計算2
%                     [obx,oby] = obodom(obx1,oby1,obx2,oby2,R1,R2);%最近障害物座標の判別
%                     obstacle = [obstacle;obx,oby];%障害物座標
%                     obstacleR = 0.3;%衝突判定用の障害物の半径0.2
                    
                    if(detect == 1 && stop == 1)%切り替え点付近のとき(DWA中に停止させるため)
                        pose=tic;
                        while(toc(pose) <= 1)%障害物が無くなるまで待機
                            com_str = sprintf("%d,290",com_s);
                            writeline(arduino,com_str);
                            velocity = 0;
                            count = 0;
                            DWA = 0;
                            DWA_go = 0;
                        end
                    end
                    
                     %行動設定
                    if(detect == 1 && DWA == 0 && DWA_go == 0 && count == 0)%一時停止措置
                        stay=tic;
                        %ここに，切り替え点付近なら一時停止し続ける指示を記載
                        if(stop == 1)%切り替え点付近のとき
                        while(toc(stay) <= 1)%障害物が無くなるまで待機
                            com_str = sprintf("%d,290",com_s);
                            writeline(arduino,com_str);
                            velocity = 0;
                            count = 0;
                        end
                        else%切り替え点付近ではないとき
                        while(toc(stay) <= 1)%1秒待機
                            com_str = sprintf("%d,290",com_s);
                            writeline(arduino,com_str);
                            velocity = 0;
                            count = 1;
                        end
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
                        %com_ac = 630;%630
%                         if(u(1,1) == 0)
%                             com_ac = 290;
%                         else
%                             com_ac = 630;
%                         end
                        %ステアリング量設定
                        com_st = round((rad2deg(asin(0.95*u(2,1)/0.27))+127.26)/0.2362);
                        %操舵角設定
                        if(com_st > 760)%760
                            com_st = 760;
                        elseif(com_st < 370)%370
                            com_st = 370;
                        end
                        %指令値
                        com_str = sprintf("%d,%d",com_st,com_ac);%発進措置
                        writeline(arduino,com_str);
                        DWA = DWA + 1;
%                     elseif(velocity == 0 && DWA == 0)%停止時
%                         com_str = sprintf("%d,290",com);
                    elseif(velocity == 1)%直進時
                        com_s = round(538.78+phi/0.2362);
                    if(com_s > 760)
                        com_s = 760;
                    elseif(com_s < 370)
                        com_s = 370;
                    end
                    if(route == 1)
                        com_a = 630;
                    elseif(route == 2)
                        com_a = 580;
                    elseif(route == 3)
                        com_a = 660;
                    elseif(route == 4)
                        com_a = 615;
                    end
                    com_str = sprintf("%d,%d",com_s,com_a);%発進措置
                    %com_str = sprintf("%d,630",com);
                    writeline(arduino,com_str);
                    end
                    
                    if (readline(arduino) == "error")
                        fprintf("error\n");
                    end
                    
                    if(flag_goal == 0)
                        goal_generate
                        %                 before_goal = goal;
                        %                 if(before_goal(1) > goal(1) || before_goal(2) > goal(2))
                        %                     goal = before_goal;
                        %                 end
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
                    %if(DWA > 0)
                    plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
                    %end
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
ylim([-25 0])
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
global obstacleR;obstacleR = 0.3;%衝突判定用の障害物の半径
%global main_goal;
global mx; mx = [Pos(1) Pos(2) deg2rad(theta) 0 0]';%ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]

[obx1,oby1,R1] = obodom1(mx);%障害物座標の計算1
[obx2,oby2,R2] = obodom2(mx);%障害物座標の計算2
[obx,oby] = obodom(obx1,oby1,obx2,oby2,R1,R2);%最近障害物座標の判別
%obstacle = [obx oby];%障害物座標
obstacle = [obstacle;obx,oby];

%ロボットの力学モデル
%[最高速度[m/s],最高回頭速度[rad/s],最高加減速度[m/ss],最高加減回頭速度[rad/ss],
% 速度解像度[m/s],回頭速度解像度[rad/s]]
Kinematic=[0.27,deg2rad(10.0),0.27,deg2rad(10.0),0.03,deg2rad(1.0)];
%Kinematic=[0.27,deg2rad(10.0),0.01,deg2rad(10.0),0.01,deg2rad(1.0)];

%評価関数のパラメータ [heading,dist,velocity,predictDT]
evalParam=[1.0,9.8,10.0,2.0];

%DWAによる入力値の計算
[u,traj]=DynamicWindowApproach(mx,Kinematic,goal,evalParam,obstacle,obstacleR);
mx=f(mx,u);%運動モデルによる移動
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

dist=2.0;%1.5
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

% if(norm(P12) <= norm(P1r))
%     flag = 1;
% else
%     flag = 0;
% end

distP = norm(P12) - norm(P1r);
if(distP <= 1.5)
    flag = 1;
else
    flag = 0;
end
end
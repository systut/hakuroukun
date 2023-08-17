mx = [0 0 deg2rad(0) 0 0]';%ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
    
scanMsg1 = rosmessage('sensor_msgs/LaserScan');

scanMsg1.Ranges = zeros(720, 1);

[obx1,oby1,R1] = obodom1(mx,scanMsg1);

disp(obx1);

disp(oby1);

disp(R1);

mx = [0 0 deg2rad(0) 0 0]';%ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
    
scanMsg = rosmessage('sensor_msgs/LaserScan');

scanMsg.Ranges = zeros(720, 1);

[obx1,oby1,R1] = obodom1(mx,scanMsg);

[obx2,oby2,R2] = obodom2(mx,scanMsg);%障害物座標の計算2

[obx,oby] = obodom(obx1,oby1,obx2,oby2,R1,R2);%最近障害物座標の判別

disp(obx);

disp(oby);

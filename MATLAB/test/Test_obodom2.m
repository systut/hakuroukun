mx = [0 0 deg2rad(0) 0 0]';%ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
    
scanMsg2 = rosmessage('sensor_msgs/LaserScan');

scanMsg2.Ranges = zeros(720, 1);

[obx2,oby2,R2] = obodom2(mx,scanMsg2);%障害物座標の計算2

disp(obx2);

disp(oby2);

disp(R2);

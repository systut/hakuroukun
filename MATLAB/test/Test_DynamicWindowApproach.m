obstacle = [obx oby];%障害物座標
obstacleR = 0.3;%衝突判定用の障害物の半径

%ロボットの力学モデル
%[最高速度[m/s],最高回頭速度[rad/s],最高加減速度[m/ss],最高加減回頭速度[rad/ss],
% 速度解像度[m/s],回頭速度解像度[rad/s]]
Kinematic=[0.27,deg2rad(10.0),0.27,deg2rad(10.0),0.03,deg2rad(1.0)];
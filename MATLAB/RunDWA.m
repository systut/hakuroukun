function [u, mx, result, obstacle] = RunDWA(dt,scanMsg1,scanMsg2, goal, Pos, theta, result)
    %global flag_goal;
    %global flag_A;
    %global main_goal;
    mx = [Pos(1) Pos(2) deg2rad(theta) 0 0]';%ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
    
    [obx1,oby1,R1] = obodom1(mx,scanMsg1);%障害物座標の計算1
    [obx2,oby2,R2] = obodom2(mx,scanMsg2);%障害物座標の計算2
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
    [u,traj]=DynamicWindowApproach(mx,Kinematic,goal,evalParam,obstacle,obstacleR,dt);
    mx=Function(mx,u,dt);%運動モデルによる移動
    
    %シミュレーション結果の保存
    result.x=[result.x; mx'];

end

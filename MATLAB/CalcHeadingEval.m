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
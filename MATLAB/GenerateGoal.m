function goal = GenerateGoal(start, plus, main_goal, mx, goal)
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
    
    goal = [x2,y2]';%DWAゴールの位置 [x(m),y(m)] DWA時に逐次更新
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
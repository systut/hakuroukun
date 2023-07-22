function [u,trajDB]=DynamicWindowApproach(mx,model,goal,evalParam,obstacle,obstacleR,dt)
    %DWAによる入力値の計算をする関数
    
    %Dynamic Window[vmin,vmax,ωmin,ωmax]の作成
    Vr=CalcDynamicWindow(mx,model,dt);
    %評価関数の計算
    [evalDB,trajDB]=Evaluation(mx,Vr,goal,obstacle,obstacleR,model,evalParam,dt);
    
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

function [evalDB,trajDB]=Evaluation(mx,Vr,goal,obstacle,obstacleR,model,evalParam,dt)
    %各パスに対して評価値を計算する関数
    evalDB=[];
    trajDB=[];
    
    for vt=Vr(1):model(5):Vr(2)
        for ot=Vr(3):model(6):Vr(4)
            %軌跡の推定
            [xt,traj]=GenerateTrajectory(mx,vt,ot,evalParam(4),model,dt);
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

function Vr=CalcDynamicWindow(mx,model,dt)
    %モデルと現在の状態からDyamicWindowを計算
    %車両モデルによるWindow
    Vs=[0 model(1) -model(2) model(2)];
    
    %運動モデルによるWindow
    Vd=[mx(4)-model(3)*dt mx(4)+model(3)*dt mx(5)-model(4)*dt mx(5)+model(4)*dt];
    
    %最終的なDynamic Windowの計算
    Vtmp=[Vs;Vd];
    Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
    %[vmin,vmax,ωmin,ωmax]
end


function [mx,traj]=GenerateTrajectory(mx,vt,ot,evaldt,model, dt)
    %軌跡データを作成する関数
    time=0;
    u=[vt;ot];%入力値
    traj=mx;%軌跡データ
    while time<=evaldt
        time=time+dt;%シミュレーション時間の更新
        mx=Function(mx,u);%運動モデルによる推移
        traj=[traj mx];
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
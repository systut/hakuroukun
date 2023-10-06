function [obx1,oby1,R1] = obodom1(mx, scanMsg1)
    %LiDARからみた障害物座標の計算1    
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

function L_phi1 = CalcLphi1(m1)
    %LiDARの角度指定1
    if (m1 <= 90)
        L_phi1 = m1*0.5;
    elseif (m1 >= 420)
        L_phi1 = (m1*0.5) - 360;
    end
end
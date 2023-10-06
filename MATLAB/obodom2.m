function [obx2,oby2,R2] = obodom2(mx,scanMsg2)
    %LiDARからみた障害物座標の計算2    
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


function L_phi2 = CalcLphi2(m2)
    %LiDARの角度指定2
    if (m2 <= 300)
        L_phi2 = m2*0.5;
    elseif (m2 >= 630)
        L_phi2 = (m2*0.5) - 360;
    end
end
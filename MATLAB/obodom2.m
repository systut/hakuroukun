function [obx2,oby2,R2] = obodom2(mx,scanMsg2)
    %LiDARからみた障害物座標の計算2    
    R2 = [scanMsg2.Ranges(1:810);  scanMsg2.Ranges(1704:1947)];
    [val,idx] = min(R2);
    m2 = idx;
    if(m2 > 810)
        m2 = m2 + 894;
    end
    
    L_phi2 = CalcLphi2(m2);
    
    R2 = R2(idx);
    
    obx2 = R2*cos(mx(3)+deg2rad(L_phi2))+(mx(1)+0.33*cos(mx(3)+deg2rad(70)));
    oby2 = R2*sin(mx(3)+deg2rad(L_phi2))+(mx(2)+0.33*sin(mx(3)+deg2rad(70)));
end


function L_phi2 = CalcLphi2(m2)
    %LiDARの角度指定2
    if (m2 <= 810)
        L_phi2 = m2*0.185;
    elseif (m2 >= 1704)
        L_phi2 = (m2*0.185) - 360;
    end
end
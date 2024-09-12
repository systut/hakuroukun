function [obx1,oby1,R1] = obodom1(mx, scanMsg1)
    % angle_increament = rad2deg(0.004623388871551);
    R1 = [scanMsg1.Ranges(1:170);  scanMsg1.Ranges(794:1360)];
    [val,idx] = min(R1);
    m1 = idx;
    if(m1 > 170)
        m1 = m1 + 624;
    end
    
    L_phi1 = CalcLphi1(m1);
    
    R1 = R1(idx);
    
    obx1 = R1*cos(mx(3)+deg2rad(L_phi1))+(mx(1)+0.33*cos(mx(3)+deg2rad(-70)));
    oby1 = R1*sin(mx(3)+deg2rad(L_phi1))+(mx(2)+0.33*sin(mx(3)+deg2rad(-70)));
end

function L_phi1 = CalcLphi1(m1)
    %LiDARの角度指定1
    if (m1 <= 170)
        L_phi1 = m1*0.2649;
    elseif (m1 >= 794)
        L_phi1 = (m1*0.2649) - 360;
    end
end
function [obx1,oby1,R1] = obodom1(mx, scanMsg1)
    % R1 = [scanMsg1.Ranges(1:243);  scanMsg1.Ranges(1623:1947)];
    % angle_increament = rad2deg(0.003228769404814);
    R1 = [scanMsg1.Ranges(1:243);  scanMsg1.Ranges(1137:1947)];
    [val,idx] = min(R1);
    m1 = idx;
    if(m1 > 243)
        m1 = m1 + 894;
    end
    
    L_phi1 = CalcLphi1(m1);
    
    R1 = R1(idx);
    
    obx1 = R1*cos(mx(3)+deg2rad(L_phi1))+(mx(1)+0.33*cos(mx(3)+deg2rad(-70)));
    oby1 = R1*sin(mx(3)+deg2rad(L_phi1))+(mx(2)+0.33*sin(mx(3)+deg2rad(-70)));
end

function L_phi1 = CalcLphi1(m1)
    %LiDARの角度指定1
    if (m1 <= 243)
        L_phi1 = m1*0.1850;
    elseif (m1 >= 1137)
        L_phi1 = (m1*0.1850) - 360;
    end
end
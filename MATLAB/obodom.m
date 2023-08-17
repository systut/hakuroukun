function [obx,oby] = obodom(obx1,oby1,obx2,oby2,R1,R2)
    %最近座標判別
    if(R1 < R2)
    obx = obx1;
    oby = oby1;
    else
    obx = obx2;
    oby = oby2;
    end
end

function [time,Pos] = ReadPosition(id,x,y,theta)
    l1 = 0.4;
    fseek(id,-500,1);
    while not(feof(id))
        ch_gps = fgetl(id);
    end
    str_gps = string(ch_gps);
    
    if(strlength(str_gps) == 144)
        Time = str2double(strsplit(strtrim(extractBetween(str_gps,12,23)),':'));
        time = Time(1)*3600+Time(2)*60+Time(3);
        str_gps = strtrim(extractBetween(str_gps,24,53));
        Pos = str2double(strsplit(str_gps));
        Pos = Pos+[l1*cos(deg2rad(theta)) l1*sin(deg2rad(theta))];
    else
        fprintf("error：%s\n",str_gps);
        time = 0;
        Pos = [x y];
    end
end

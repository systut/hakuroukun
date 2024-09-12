function [time,Pos] = ReadPosition2(port)
    gps = gpsdev(port,'OutputFormat',"timetable");
    [tt,~] = read(gps);
    lat = tt.LLA(1);
    lon = tt.LLA(2);
    alt = tt.LLA(3);
    
    origin = [34.702517698333330, 1.374089479466667e+02, 80.163000000000000];
    [xEast,yNorth,~] = latlon2local(lat,lon,alt,origin);
    time = tt.Time.Hour*3600 + tt.Time.Minute*60 + tt.Time.Second;
    Pos = [xEast yNorth];
end


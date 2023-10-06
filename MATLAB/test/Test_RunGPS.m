port = 'COM9';

gps = gpsdev(port,'OutputFormat',"timetable");

[tt,overruns] = read(gps);

disp(tt.LLA)

disp("Lattitude: "+ tt.LLA(1));
lat = tt.LLA(1);
disp("Longitude: "+ tt.LLA(2));
lon = tt.LLA(2);
disp("Altitude: "+ tt.LLA(3));
alt = 80.163000000000000;
origin = [34.702517698333330, 1.374089479466667e+02, 80.163000000000000];

clear gps

[xEast,yNorth,zUp] = latlon2local(lat,lon,alt,origin);
fprintf("x,y: %f,%f \n", xEast, yNorth);
port = 'COM4';

gps = gpsdev(port,'OutputFormat',"timetable");

[tt,overruns] = read(gps);

disp(tt.LLA)

disp("Lattitude: "+ tt.LLA(0));

disp("Lattitude: "+ tt.LLA(1));
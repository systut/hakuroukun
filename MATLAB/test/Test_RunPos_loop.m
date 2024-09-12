x = 0.0;
y = 0.0;
theta = 0.0;
theta_0 = 97.0;
rz_offset = 0.0;
offset_count = 0;

% Device GPS
port = 'COM9';
gps = gpsdev(port,'OutputFormat',"timetable");

% Device_IMU
tcp = tcpclient("localhost",10000);
configureTerminator(tcp,"LF","CR/LF");
flush(tcp);
writeline(tcp,"start");

flush(tcp)
tic
while(toc < 10)
    if(tcp.NumBytesAvailable >= 10)
        theta = theta_0 + ReadTheta(tcp,theta,rz_offset);
        offset_count = offset_count+1;
    end
end
rz_offset = theta/(offset_count/(1/0.15))*100;
theta = theta_0;

while true
    
    [tt,overruns] = read(gps);
%     disp(tt.LLA)
    
%     disp("Lattitude: "+ tt.LLA(1));
    lat = tt.LLA(1);
%     disp("Longitude: "+ tt.LLA(2));
    lon = tt.LLA(2);
%     disp("Altitude: "+ tt.LLA(3));
    alt = tt.LLA(3);
    
    origin = [34.702517698333330, 1.374089479466667e+02, 80.163000000000000];

    [xEast,yNorth,zUp] = latlon2local(lat,lon,alt,origin);
%     fprintf("Current Pos : %f, %f \n", xEast,yNorth);
    
    if(tcp.NumBytesAvailable >= 10)
        theta = rad2deg(angdiff(0,deg2rad(ReadTheta(tcp,theta,rz_offset))));
    end
    
    fprintf("Current Pos : %f \n", theta);
end
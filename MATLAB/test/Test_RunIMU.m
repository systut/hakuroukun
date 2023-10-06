theta = 0.0;
theta_0 = 97.0;
rz_offset = 0.0;
offset_count = 0;

% Device_IMU
tcp = tcpclient("localhost",10000);
configureTerminator(tcp,"LF","CR/LF");
flush(tcp);
writeline(tcp,"start");

% Get IMU Data 0 -> Initial Theta_0
tic
while (toc < 1)
end

flush(tcp)
tic
while(toc < 10)
    if(tcp.NumBytesAvailable >= 10)
        theta = ReadTheta(tcp,theta,rz_offset);
        offset_count = offset_count+1;
    end
end
rz_offset = theta/(offset_count/(1/0.15))*100;
theta = theta_0;

while true 
%     flush(tcp)
    if(tcp.NumBytesAvailable >= 10)
        theta = rad2deg(angdiff(0,deg2rad(ReadTheta(tcp,theta,rz_offset))));
        fprintf("Theta : %f\n", theta);
        flush(tcp)
        tic
        while (toc < 2)
        end
           
    end
end
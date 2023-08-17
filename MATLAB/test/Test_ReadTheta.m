tcp = tcpclient("localhost",10000);

configureTerminator(tcp,"LF","CR/LF");

flush(tcp);

writeline(tcp,"start");

theta = 0;

rz_offset = 0.0;

% theta = ReadTheta(tcp,theta,rz_offset);

tic
while(toc < 10)
    if(tcp.NumBytesAvailable >= 10)
        theta = ReadTheta(tcp,theta,rz_offset);
    end
end
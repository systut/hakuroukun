x = 0.0;

y = 0.0;

theta = 0.0;

id = fopen('data\gps','r');

[time,Pos] = ReadPosition(id,x,y,theta);


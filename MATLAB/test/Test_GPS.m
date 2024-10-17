clear
clc

[pub, sub] = InitROS();

tic
while (toc < 1)
end
theta0 = 0.0;

while true
    theta = theta0 + GetTheta(sub.imu);
    Pos = GetPosition(sub.gps, theta);
    fprintf("Position : %f, %f \n", Pos(1), Pos(2));
end


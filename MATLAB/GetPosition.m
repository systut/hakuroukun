function Pos = GetPosition(gps_subscriber, theta)
    l1 = 0.4;
    x = gps_subscriber.LatestMessage.Pose.Position.X;
    y = gps_subscriber.LatestMessage.Pose.Position.Y;
    x_fw = x + l1*cos(theta);
    y_fw = y + l1*sin(theta);
    Pos = [x_fw y_fw];
    fprintf("%f, %f\n", Pos(1), Pos(2));
end
function theta = GetTheta(imu_subscriber)
    theta = [imu_subscriber.LatestMessage.Data];
    % fprintf("%f\n", theta);
end
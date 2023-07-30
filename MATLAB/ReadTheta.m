function ret = ReadTheta(tcp,theta,rz_offset)
    str_imu = readline(tcp);
    str_imu = erase(str_imu,char(0));
    
    if(startsWith(str_imu,"ags,"))
        str_imu = erase(str_imu,"ags,");
        data_imu = str2double(strsplit(str_imu,','));
        rot_z = (data_imu(7)-rz_offset)/100*0.15;
        theta = theta+rot_z;
    else
        fprintf("message:%s\n",str_imu);
    end
    ret = theta;
end
function [com_s,com_a] = VelocityControl(phi)
    com_s = round(538.78+phi/0.2362);%538.78
    if(com_s > 760)%760
        com_s = 760;
    elseif(com_s < 370)%370
        com_s = 370;
    end
    
    com_a = 600;
    if(com_a > 680)
        com_a = 680;
    elseif (com_a < 580)%290
        com_a = 580;%290
    end
end
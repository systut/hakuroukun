function [phi,flag] = CalculatePhi(P1,P2,param,x,y,theta,distp2)
    a = param(1);
    b = param(2);
    c = param(3);
    kp = 1.0;
    n = [a b];
    Pn = [x y];
    
    d = (a*Pn(1)+b*Pn(2)+c)/norm(n);
    dis = abs(d);
    Pr = Pn-d.*normr(n);
    
    %P12<P1rでbreak
    P12 = P2-P1;
    P1r = Pr-P1;
    
    theta_r = rad2deg(atan2(P12(2),P12(1)));
    P1c = Pn-P1;
    sgn = sign(cross([P1c 0],[P12 0]));
    beta = rad2deg(atan(kp*sgn(3)*dis));
    phi = rad2deg(angdiff(deg2rad(theta-theta_r),deg2rad(beta)));
    
    distP = norm(P12) - norm(P1r);
    if(distP <= distp2)
        flag = 1;
    else
        flag = 0;
    end
end
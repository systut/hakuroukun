function mx = Function(mx,u,dt)
    % Motion Model    
    F = [1 0 0 0 0
        0 1 0 0 0
        0 0 1 0 0
        0 0 0 0 0
        0 0 0 0 0];
    
    B = [dt*cos(mx(3)) 0
        dt*sin(mx(3)) 0
        0 dtf
        1 0
        0 1];
    
    mx= F*mx+B*u;
    if(mx(3) >= 3*pi/2)
        mx(3) = mx(3)-2*pi;
    end
end

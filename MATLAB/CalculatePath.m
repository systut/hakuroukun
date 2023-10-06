function [a,b,c] = CalculatePath(P1,P2)
    A = [P1(1) 1; P2(1) 1];
    B = [P1(2); P2(2)];
    
    if(P1(1) == P2(1))
        a = 1;
        b = 0;
        c = -P1(1);
        line_y = linspace(P1(2),P2(2),100);
        line_x = -c;
    else
        X = A\B;
        a = X(1);
        b = -1;
        c = X(2);
        line_x = linspace(P1(1),P2(1),100);
        line_y = a*line_x+c;
    end
    plot(line_x,line_y,'Color','r');
end
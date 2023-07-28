% P1 = [0; 0];
% 
% P2 = [1; 1];

% [a, b, c] = CalculatePath(P1, P2);
% 
% disp(a);
% 
% disp(b);
% 
% disp(c);

P1 = [2; 2];

P2 = [3; 3];

A = [P1(1) 1; P2(1) 1];
B = [P1(2); P2(2)];

X = A\B;
a = X(1);
b = -1;
c = X(2);
line_x = linspace(P1(1),P2(1),100);
line_y = a*line_x+c;
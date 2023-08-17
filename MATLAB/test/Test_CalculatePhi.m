x = 0.0;
y = 0.0;
theta = 0.0;

P1 = [0; 0];

P2 = [1; 1];

distp2 = 1.5;

[a, b, c] = CalculatePath(P1, P2);
param(1,:) = [a b c];

phi = @(x,y,theta)CalculatePhi(P1,P2,param(1,:),x,y,theta,distp2);

disp(phi);

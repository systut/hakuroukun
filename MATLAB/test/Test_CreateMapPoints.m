clear
clc
close all

% Input
length = 4;
width = 4;
n_width = 3;

% Output
[Points, n] = CreateMapPoints(length, width, n_width);
% plot(P(:,1),P(:,2),'o','MarkerSize',10)

global Pos; Pos = [0 0];
global P;

for i=1:n
    P_i = Pos + Points(i,:);
    P(i,:) = P_i;
end

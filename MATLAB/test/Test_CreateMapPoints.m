clear
clc
close all

% Input
length = 14;
width = 30;
n_width = 1.5;

% Output
[P, n] = CreateMapPoints(length, width, n_width);
plot(P(:,1),P(:,2),'o','MarkerSize',10)